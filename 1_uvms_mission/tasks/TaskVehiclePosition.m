classdef TaskVehiclePosition < Task
    % Exercise 1: 
    % Implement a vehicle position control task, and test that the vehicle reaches the required position.
    % Add the proper Jacobian variable, its activation function, and the desired reference rate.
    
    properties

    end


    methods
        function updateReference(obj, robot)
            [~, lin] = CartError(robot.wTgv , robot.wTv);
            obj.xdotbar = - 0.8 * robot.vTw(1:3,1:3) * lin;
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.8);
        end

        function updateJacobian(obj, robot)
            obj.J = [zeros(3,7) -eye(3) zeros(3)];
        end

        function updateActivation(obj, robot)
            obj.A = eye(3);
        end
    end
end