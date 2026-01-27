classdef ToolAltitudeLimit < Task
    % ToolAltitudeLimit per Esercizio 3 (Decentralized)
    % Mantiene l'altitudine minima del Tool per il SINGOLO robot.
    
    properties
        z_min = 0.15;       % Limite inferiore altezza (m)
        z_buff = 0.05;      % Buffer di attivazione (m)
        gain = 1.0;         % Guadagno proporzionale
        smooth_active = true;
    end

    methods
        function obj = ToolAltitudeLimit(robID, taskID, smooth_flag)
            obj.ID = robID;
            obj.task_name = taskID;
            if nargin >= 3
                obj.smooth_active = smooth_flag;
            end
        end

        function updateReference(obj, robot)
            % INPUT: 'robot' è la struttura del SINGOLO braccio (7 DOF)
            
            % Lettura Altitudine Tool (Componente Z della posizione)
            % wTt è 4x4, l'elemento (3,4) è la z traslazionale.
            curr_alt = robot.wTt(3,4);

            % Controllo difensivo sui dati
            if isscalar(curr_alt)
                % Calcolo errore: positivo se siamo sotto il limite (es. 0.15 - 0.10 = +0.05)
                err = obj.z_min - curr_alt;
                
                % Logica "Push Away": agiamo solo se l'errore è positivo (siamo troppo bassi)
                % Il gain positivo genera una velocità positiva (+Z) per risalire.
                raw_vel = obj.gain * max(0, err);
                
                obj.xdotbar = raw_vel;
            else
                obj.xdotbar = 0;
            end

            % Saturazione di sicurezza (m/s)
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end

        function updateJacobian(obj, robot)
            % INPUT: 'robot' è il singolo manipolatore
            
            % Jacobiano geometrico del tool (6x7)
            J_tool = robot.wJt;

            % Vettore di selezione.
            % In base alla convenzione usata in ObjectTask [Angular; Linear]:
            % 1-3: Velocità Angolari (wx, wy, wz)
            % 4-6: Velocità Lineari (vx, vy, vz)
            % Quindi per controllare l'altitudine (vz) selezioniamo il 6° elemento.
            select_vec = [0 0 0 0 0 1];

            % Proiezione del Jacobiano sulla sola riga Z (risultato 1x7)
            % Non servono zeri aggiuntivi perché siamo nel caso decentralizzato (7 dof).
            obj.J = select_vec * J_tool;
        end

        function updateActivation(obj, robot)
            curr_alt = robot.wTt(3,4);

            % Attivazione soft:
            % 1 (piena attivazione) se siamo sotto z_min
            % 0 (disattivato) se siamo sopra z_min + buffer
            if isscalar(curr_alt) && obj.smooth_active
                obj.A = DecreasingBellShapedFunction(obj.z_min, ...
                                                     obj.z_min + obj.z_buff, ...
                                                     0, 1, curr_alt);
            elseif isscalar(curr_alt) && ~obj.smooth_active
                 % Attivazione hard (binaria) se richiesto
                 if curr_alt < obj.z_min
                     obj.A = 1;
                 else
                     obj.A = 0;
                 end
            else
                obj.A = 0;
            end
        end
    end
end