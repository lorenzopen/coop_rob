classdef rigid_constraint_task < Task
    % Bimanual rigid grasp kinematic constraint
    % Enforces: J_oL*qdot_L - J_oR*qdot_R = 0

    methods
        function obj = rigid_constraint_task(robotID, taskID)
            obj.ID = robotID;
            obj.task_name = taskID;
        end
        
        function updateReference(obj, robot_system)
            % Rigid constraint - zero reference
            obj.xdotbar = zeros(6,1);
        end
        
        function updateJacobian(obj, robot_system)        
            % Object-frame Jacobians
            J_L = robot_system.left_arm.wJt;  
            J_R = robot_system.right_arm.wJt;              
            obj.J = [J_L, -J_R];
        end
        
        function updateActivation(obj, robot_system)
            % Physical constraint - always active
            obj.A = eye(6);
        end
    end
end
