classdef rigid_constraint_task < Task
    % Bimanual rigid grasp kinematic constraint
    % Enforces: J_oL*qdot_L - J_oR*qdot_R = 0

    methods
        function obj = rigid_constraint_task(robotID, taskID)
            obj.ID = robotID;
            obj.task_name = taskID;
        end
        
        function updateReference(obj, robot)
            % Rigid constraint - zero reference
            obj.xdotbar = zeros(6,1);
        end
        
        function updateJacobian(obj, robot)             
            obj.J = robot.wJt;
        end
        
        function updateActivation(obj, robot)
            % Physical constraint - always active
            obj.A = eye(6);
        end
    end
end
