classdef ObjectTask < Task
    % ObjectTask per Esercizio 3 (Cooperative - Decentralized)
    % Calcola il riferimento di velocità per il frame dell'oggetto e
    % lo Jacobiano corrispondente per un SINGOLO braccio (7 DOF).
    
    properties
        gain = 1.0;
    end

    methods
        function obj = ObjectTask(robot_ID, taskID)
            obj.ID = robot_ID;
            obj.task_name = taskID;
        end

        function updateReference(obj, robot)
            % INPUT: 'robot' è la struttura del SINGOLO braccio
            % Non serve più selezionare tra left/right basandosi sull'ID
            
            % 1. Calcolo dell'errore Cartesiano
            % wTog: Goal pose dell'oggetto (Global)
            % wTo: Posa attuale dell'oggetto (che deve essere aggiornata nel main loop)
            [v_ang, v_lin] = CartError(robot.wTog, robot.wTo);

            % Log interno per debug (se supportato dalla struttura robot)
            % robot.dist_to_goal = norm(v_lin);
            % robot.rot_to_goal = norm(v_ang);

            % 2. Calcolo velocità desiderata (Reference)
            % Convenzione: [v_ang; v_lin]
            obj.xdotbar = obj.gain * [v_ang; v_lin];

            % 3. Saturazione (Safety)
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.3); % Rad/s
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.3); % m/s
        end

        function updateJacobian(obj, robot)
            % INPUT: 'robot' è il singolo manipolatore
            
            % 1. Jacobiano geometrico alla flangia/pinza (6x7)
            J_tool = robot.wJt;

            % 2. Calcolo della trasformazione spaziale
            % Calcoliamo il vettore r_toc: distanza tra Tool e Oggetto nel frame World
            % wTt = posa tool, wTo = posa oggetto
            r_toc = robot.wTo(1:3, 4) - robot.wTt(1:3, 4);

            % Matrice antisimmetrica del vettore r_toc
            S_r = skew(r_toc); 
            
            % Matrice di trasformazione per portare la velocità dal tool all'oggetto
            % Convenzione [ang; lin]:
            % omega_obj = omega_tool
            % v_obj = v_tool + omega_tool x r_toc = v_tool - r_toc x omega_tool
            %       = v_tool + S_r * omega_tool (Nota il segno dipende dalla definizione di skew)
            %
            % La matrice di trasporto corretta per [omega; v] è:
            % [ I     0 ]
            % [ -S_r  I ]
            
            wS_toc = [eye(3), zeros(3); 
                      -S_r,   eye(3)];

            % 3. Jacobiano proiettato sull'oggetto (6x7)
            % In un sistema decentralizzato, NON aggiungiamo zeri per l'altro robot.
            % Questo Jacobiano lega le q_dot del singolo robot alla v_obj.
            obj.J = wS_toc * J_tool;
        end

        function updateActivation(obj, robot)
            % Attivazione sempre presente per il task principale
            obj.A = eye(6);
        end
    end
end