classdef StopTask < Task
    % StopTask  (Decentralized)
    % Impone velocità zero a tutti i giunti del SINGOLO robot.
    
    properties
    end
    
    methods
        function obj = StopTask(robot_ID, taskID)
            obj.ID = robot_ID;
            obj.task_name = taskID;
        end
        
        function updateReference(obj, robot)
            
            % vel des = 0
            obj.xdotbar = zeros(7, 1);
        end
        
        function updateJacobian(obj, robot)
            
            obj.J = eye(7);
        end
        
        function updateActivation(obj, robot)
            % Attivazione totale su tutti i giunti
            obj.A = eye(7);
        end
    end
end