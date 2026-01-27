classdef RigidConstrTask < Task
    % RigidClosedChainConstraint
    % Impone che le due braccia si muovano in modo coordinato rispettando 
    % la rigidità dell'oggetto afferrato.
    % Equazione: J_obj_L * qdot_L - J_obj_R * qdot_R = 0

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

            % --- CALCOLO JACOBIANO DESTRO TRASPORTATO (J_oR) ---
            % Distanza Tool R -> Oggetto
            r_R = robot_R.wTo(1:3, 4) - robot_R.wTt(1:3, 4);
            S_R = skew(r_R);
            % Grasp Matrix Destra
            X_R = [eye(3), zeros(3); -S_R, eye(3)];
            % Jacobiano R proiettato sull'oggetto
            J_oR = X_R * robot_R.wJt;

            % --- COSTRUZIONE MATRICE DI VINCOLO (6x14) ---
            % La differenza di velocità deve essere zero: V_L - V_R = 0
            % Quindi la matrice è [J_oL, -J_oR]
            obj.J = [J_oL, -J_oR];
        end
        
        function updateActivation(obj, robot_system)
            % Sempre attivo
            obj.A = eye(6);
        end
    end
end