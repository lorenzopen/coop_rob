%% 
classdef TaskVehicleLand < Task   
    properties
    end

    methods
        function updateReference(obj, robot)
            if isempty(robot.altitude)
                alt = 2;
            else
                alt = robot.altitude;
            end
                      
            obj.xdotbar = - 0.4 * (alt-0.5);
            obj.xdotbar = Saturate(obj.xdotbar, 0.5);

        end

        function updateJacobian(obj, robot)
            % Jacobian maps z-velocity to altitude change
            % [0 0 1] selects z component of linear velocity
            obj.J = [0 0 1] * [zeros(3,7), eye(3), zeros(3,3)];
        end
        
        function updateActivation(obj, robot)
              obj.A = 1;
        end
    end
end