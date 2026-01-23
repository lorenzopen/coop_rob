classdef TaskStopVehicle < Task
    methods
        function updateReference(obj, robot)
            obj.xdotbar = zeros(6,1); % 0 velocity
        end

        function updateJacobian(obj, robot)
            
            J_arm = zeros(6,7);
            J_vehicle = [eye(6)];

            obj.J = [J_arm, J_vehicle]; 
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(6); 
        end
    end
end