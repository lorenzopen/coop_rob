classdef ToolAltitudeLimit < Task
    % Task ibrida: Mantiene l'altitudine minima del Tool.
    % Include controlli di sicurezza sui dati in ingresso.
    
    properties
        z_min = 0.15;       % Limite inferiore altezza (m)
        z_buff = 0.05;      % Buffer di attivazione (m)
        gain = 1.0;         % Guadagno proporzionale
        smooth_active;      % Flag ereditato dal costruttore
    end

    methods
        function obj = ToolAltitudeLimit(robID, taskID, smooth_flag)
            obj.ID = robID;
            obj.task_name = taskID;
            if nargin < 3
                obj.smooth_active = true;
            else
                obj.smooth_active = smooth_flag;
            end
        end

        function updateReference(obj, sys)
            % Selezione braccio
            if obj.ID == 'L'
                rob = sys.left_arm;
            else
                rob = sys.right_arm;
            end

            % Lettura Altitudine Tool (Componente Z della posizione)
            % wTt è 4x4, elemento (3,4) è la z.
            curr_alt = rob.wTt(3,4);

            % Controllo difensivo sui dati (preso dal tuo esempio)
            if isscalar(curr_alt)
                % Calcolo errore: positivo se siamo sotto il limite (0.15 - 0.1 = 0.05)
                err = obj.z_min - curr_alt;
                
                % Logica "Push Away": agiamo solo se l'errore è positivo
                % Nota: gain positivo per spingere in alto (+Z)
                raw_vel = obj.gain * max(0, err);
                
                obj.xdotbar = raw_vel;
            else
                % Se i dati non sono validi, fermiamo la task
                obj.xdotbar = 0;
            end

            % Saturazione di sicurezza
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end

        function updateJacobian(obj, sys)
            if obj.ID == 'L'
                rob = sys.left_arm;
            else
                rob = sys.right_arm;
            end
            
            % Jacobiano geometrico del tool (6x7)
            J_tool = rob.wJt;

            % Vettore di selezione. 
            % [0 0 1 0 0 0] seleziona la velocità LINEARE Z (altitudine).
            % [0 0 0 0 0 1] selezionerebbe la rotazione (sbagliato per l'altitudine).
            select_vec = [0 0 1 0 0 0];

            % Proiezione del Jacobiano sulla sola riga Z
            J_z = select_vec * J_tool;

            % Costruzione Jacobiano aumentato (1x14)
            if obj.ID == 'L'
                obj.J = [J_z, zeros(1, 7)];
            else
                obj.J = [zeros(1, 7), J_z];
            end
        end

        function updateActivation(obj, sys)
            if obj.ID == 'L'
                rob = sys.left_arm;
            else
                rob = sys.right_arm;
            end

            curr_alt = rob.wTt(3,4);

            % Attivazione soft:
            % 1 (piena attivazione) se siamo sotto z_min (0.15)
            % 0 (disattivato) se siamo sopra z_min + buffer (0.20)
            if isscalar(curr_alt)
                obj.A = DecreasingBellShapedFunction(obj.z_min, ...
                                                     obj.z_min + obj.z_buff, ...
                                                     0, 1, curr_alt);
            else
                obj.A = 0;
            end
        end
    end
end