%% 
classdef TaskLand < Task   
    properties
    end

    methods
        function updateReference(obj, robot)
            if isempty(robot.altitude)
                altitude = 1.0;   % fallback value
            else
                altitude = robot.altitude;
            end
                      
            obj.xdotbar = 0.6 * (0.5 - altitude);
            obj.xdotbar = Saturate(obj.xdotbar, 0.5);

        end

        function updateJacobian(obj, robot)
            
            n = [0 0 1];
            J_arm = zeros(3,7);
            J_vehicle = [eye(3) zeros(3,3)];

            obj.J = n * [J_arm, J_vehicle];
        end
        
        function updateActivation(obj, robot)
              obj.A = 1;
        end
    end
end