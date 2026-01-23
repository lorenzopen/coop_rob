classdef stop_velocities_task < Task   
    properties

    end

    methods
        function obj=stop_velocities_task(robot_ID,taskID)
            obj.ID=robot_ID;
            obj.task_name=taskID;
        end
        function updateReference(obj, robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;   
            end
            
            error = zeros(7, 1) - robot.qdot; % desired velocity is zero
            obj.xdotbar = 0.5 * error;

            % limit the requested velocities
            obj.xdotbar = Saturate(obj.xdotbar, 0.3);
        end

        function updateJacobian(obj, robot_system)            
            if obj.ID=='L'
                obj.J=[eye(7), zeros(7, 7)];
            elseif obj.ID=='R'
                obj.J=[zeros(7, 7), eye(7)];
            end
        end

        function updateActivation(obj, robot_system)
            obj.A = eye(7);
        end
    end
end