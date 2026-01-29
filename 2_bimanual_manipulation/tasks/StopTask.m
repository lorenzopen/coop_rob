classdef StopTask < Task
    % Task unificato per arrestare le velocità dei giunti (singolo braccio o entrambi)
    
    properties
        % Eventuali proprietà aggiuntive possono essere rimosse se non usate
    end
    
    methods
        function obj = StopTask(robot_ID, taskID)
            % robot_ID può essere 'L', 'R' (7 DOF) o 'Both' (14 DOF)
            obj.ID = robot_ID;
            obj.task_name = taskID;
        end
        
        function updateReference(obj, robot_system)
            if strcmp(obj.ID, 'L')
                robot_qdot = robot_system.left_arm.qdot;
                target_size = 7;
            elseif strcmp(obj.ID, 'R')
                robot_qdot = robot_system.right_arm.qdot;
                target_size = 7;
            else % Assume 'Both' o default a 14 DOF
                robot_qdot = [robot_system.left_arm.qdot; robot_system.right_arm.qdot];
                target_size = 14;
            end
            
            % Calcolo dell'errore (Desiderata [0] - Attuale)
            error = zeros(target_size, 1) - robot_qdot;
            
           
            obj.xdotbar = 0.5 * error;
            
            % Limita le velocità richieste per sicurezza
            obj.xdotbar = Saturate(obj.xdotbar, 0.3);
        end
        
        function updateJacobian(obj, robot_system)
            switch obj.ID
                case 'L'
                    obj.J = [eye(7), zeros(7, 7)];
                case 'R'
                    obj.J = [zeros(7, 7), eye(7)];
                otherwise % 'Both'
                    obj.J = eye(14);
            end
        end
        
        function updateActivation(obj, robot_system)
            if size(obj.J, 1) == 7
                obj.A = eye(7);
            else
                obj.A = eye(14);
            end
        end
    end
end