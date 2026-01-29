classdef RigidConstrTask < Task


    properties
        % Flag per indicare al solver che questo è un vincolo rigido (HARD constraint)
        is_kin_constraint = true; 
    end

    methods
        function obj = RigidConstrTask(robotID, taskID)
            obj.ID = robotID;
            obj.task_name = taskID;
        end
        
        function updateReference(obj, robot_system)
            % La velocità relativa tra i due punti calcolati dell'oggetto deve essere zero
            obj.xdotbar = zeros(6,1);
        end
        
        function updateJacobian(obj, robot_system)        
            % Recuperiamo i robot
            robot_L = robot_system.left_arm;
            robot_R = robot_system.right_arm;

            % --- CALCOLO JACOBIANO SINISTRO TRASPORTATO (J_oL) ---
            % Distanza Tool L -> Oggetto
            r_L = robot_L.wTo(1:3, 4) - robot_L.wTt(1:3, 4);
            S_L = skew(r_L);
            % Grasp Matrix Sinistra
            X_L = [eye(3), zeros(3); -S_L, eye(3)]; 
            % Jacobiano L proiettato sull'oggetto
            J_oL = X_L * robot_L.wJt;

            
            % Distanza Tool R -> Oggetto
            r_R = robot_R.wTo(1:3, 4) - robot_R.wTt(1:3, 4);
            S_R = skew(r_R);
            % Grasp Matrix Destra
            X_R = [eye(3), zeros(3); -S_R, eye(3)];
            % Jacobiano R proiettato sull'oggetto
            J_oR = X_R * robot_R.wJt;

          
            obj.J = [J_oL, -J_oR];
        end
        
        function updateActivation(obj, robot_system)
           
            obj.A = eye(6);
        end
    end
end