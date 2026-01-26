classdef TaskAltitude < Task
 
    properties
            %minimum Altitude desired
            altmin = 3.0;
            %gain (lambda)
            kp = 0.5;
            error
    end
 
    methods
        function updateReference(obj,robot)
            %I want to control the Altitude
            if isempty(robot.altitude)
                altitude = 1.0;   % fallback value
            else
                altitude = robot.altitude;
            end
 
            %find the error to control the velocity
            error = altitude - obj.altmin;
           
            %control variable
            obj.xdotbar = -obj.kp * error;
            obj.xdotbar = Saturate(obj.xdotbar,0.5);
            obj.error = error;
        end
 
        function updateJacobian(obj,robot)
            %the arm isn't involved
            J_arm = zeros(3,7);
 
            %vehicle angular not involved , vehicle linar involved
            J_vehicle = [eye(3) zeros(3,3)];
 
            %total jacobian, first we normalize to obtain only the linear
            %part on the z-axis
            n = [0 0 1];
            obj.J = n * [J_arm , J_vehicle]; %1x3 * 3x13  = 1x13
 
        end
 
        function updateActivation(obj,robot)
              if isempty(robot.altitude)
                altitude = 1.0;   % fallback value
              else
                altitude = robot.altitude;
              end
            
            obj.A = DecreasingBellShapedFunction(2.0, 3.0, 0, 1, altitude);
 
        end
    end
end