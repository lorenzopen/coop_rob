classdef object_motion_task < Task
    % Object Cartesian motion task (after grasp)

    methods
        function obj = object_motion_task(robot_ID,taskID)
            obj.ID=robot_ID;
            obj.task_name = taskID;
        end
        
        function updateReference(obj, robot)
            [v_ang, v_lin] = CartError(robot.wTog, robot.wTo);

            % Desired object velocity
            obj.xdotbar = 1.0*[v_ang; v_lin];

            % Saturation
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.3);
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.3);
        end
        
        function updateJacobian(obj, robot)
            obj.J=robot.wJt;
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(6);
        end
    end
end
