classdef ObjectTask < Task
    % ObjectTaskAdvanced
   

    properties
        gain = 1.0;
    end

    methods
        function obj = ObjectTask(robot_ID, taskID)
            obj.ID = robot_ID;
            obj.task_name = taskID;
        end

        function updateReference(obj, robot_system)
            % 1. Selezione del braccio corretto
            if(obj.ID == 'L')
                robot = robot_system.left_arm;
            elseif(obj.ID == 'R')
                robot = robot_system.right_arm;
            end

            % 2. Calcolo dell'errore
       
            [v_ang, v_lin] = CartError(robot.wTog, robot.wTo);

            % Log interno per debug/grafici
            robot.dist_to_goal = norm(v_lin);
            robot.rot_to_goal = norm(v_ang);

            % 3. Calcolo velocità desiderata (Reference)
            obj.xdotbar = obj.gain * [v_ang; v_lin];

            % 4. Saturazione (Safety)
            % Saturiamo linearmente e angolarmente separatamente per stabilità
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.3); % Rad/s
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.3); % m/s
        end

        function updateJacobian(obj, robot_system)
            % 1. Selezione braccio e Jacobiano al Tool (wJt)
            if(obj.ID == 'L')
                robot = robot_system.left_arm;
            elseif(obj.ID == 'R')
                robot = robot_system.right_arm;
            end
            
            % Jacobiano geometrico alla flangia/pinza (6x7)
            J_tool = robot.wJt;

            % 2. Calcolo della trasformazione spaziale (La parte "Smart")
            % Calcoliamo il vettore r_toc: distanza tra Tool e Oggetto nel frame World
            % wTt = posa tool, wTo = posa oggetto
            r_toc = robot.wTo(1:3, 4) - robot.wTt(1:3, 4);

            % Creiamo la matrice di trasporto delle velocità (6x6)
            % [ I      0 ]
            % [ -rx    I ]  (Nota: dipende dalla convenzione [ang; lin] o [lin; ang])
            % Qui assumiamo xdot = [v_ang; v_lin] come nel riferimento sopra.
            
            % Matrice antisimmetrica del vettore r_toc
            S_r = skew(r_toc); 
            
            % Matrice di trasformazione per portare la velocità dal tool all'oggetto
            % Se ruoto il polso, l'oggetto trasla -> termine S_r
            wS_toc = [eye(3), zeros(3); 
                      -S_r,   eye(3)];

            % Jacobiano proiettato sull'oggetto (6x7)
            J_obj_local = wS_toc * J_tool;

            % 3. Costruzione Jacobiano Globale (La parte "System Compliant")
            % Espandiamo a 6x14 per gestire l'intero robot (L+R)
            if obj.ID == 'L'
                % [ J_Left_Transformed,  Zeros_Right ]
                obj.J = [J_obj_local, zeros(6, 7)];
            elseif obj.ID == 'R'
                % [ Zeros_Left,  J_Right_Transformed ]
                obj.J = [zeros(6, 7), J_obj_local];
            end
        end

        function updateActivation(obj, robot_system)
            obj.A = eye(6);
        end
    end
end