%% 
classdef TaskVehicleAltitude < Task   
    properties
        
    end

    methods
        function updateReference(obj, robot)
            if isempty(robot.altitude)
                alt = 3.0;
            else
                alt = robot.altitude;
            end
                      
            obj.xdotbar = 0.6 * (3.0 - alt);
            obj.xdotbar = Saturate(obj.xdotbar, 0.6);

        end

        function updateJacobian(obj, robot)
            % Jacobian maps z-velocity to altitude change
            % [0 0 1] selects z component of linear velocity
            obj.J = [0 0 1] * [zeros(3,7), eye(3), zeros(3,3)];
        end
        
        function updateActivation(obj, robot)
            % Activate when close to seafloor
            % Fully active when altitude < 2.0m
            % Start activating when altitude < 3.0m
            % Compute scalar activation
            if isempty(robot.altitude)
                alt = 3.0;
            else
                alt = robot.altitude;
            end

            obj.A = DecreasingBellShapedFunction(2.0, 3.0, 0, 1, alt);
        end
    end
end
