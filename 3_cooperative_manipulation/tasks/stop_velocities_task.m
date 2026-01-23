classdef stop_velocities_task < Task   
    properties

    end

    methods
        function obj=stop_velocities_task(robot_ID,taskID)
            obj.ID=robot_ID;
            obj.task_name=taskID;
        end
        function updateReference(obj, robot)
            error = zeros(7, 1) - robot.qdot; % desired velocity is zero
            obj.xdotbar = 0.5 * error;

            % limit the requested velocities
            obj.xdotbar = Saturate(obj.xdotbar, 0.3);
        end

        function updateJacobian(obj, robot)            
            obj.J=eye(7);
        end

        function updateActivation(obj, robot)
            obj.A = eye(7);
        end
    end
end