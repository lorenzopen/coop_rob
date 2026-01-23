classdef coop_tool_velocity_task < Task
    % Non-reactive cooperative tool velocity tracking

    methods
        function obj = coop_tool_velocity_task(robotID, taskID)
            obj.ID = robotID;
            obj.task_name = taskID;
        end

        function updateReference(obj, robot)
            % NON-reactive: reference comes from cooperation layer
            obj.xdotbar = robot.xdot_coop;   % 6x1
        end

        function updateJacobian(obj, robot)
            obj.J = robot.wJt;
        end

        function updateActivation(obj, robot)
            obj.A = eye(6);
        end
    end
end
