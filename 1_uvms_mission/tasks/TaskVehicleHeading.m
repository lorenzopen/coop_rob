classdef TaskVehicleHeading < Task   
    methods
        function updateReference(obj, robot)
            % Vehicle x-axis
            w_xv = robot.wTv(1:3,1);

            % Nodule position
            nodule_pos = robot.wTg(1:3,4);

            % Vector from vehicle to nodule (horizontal plane)
            diff = nodule_pos - robot.wTv(1:3,4);
            diff(3) = 0; % neglect vertical component
            if norm(diff) > 1e-3
                w_xd = diff / norm(diff);
            else
                w_xd = w_xv;  % avoid singularity
            end

            % Rotation axis
            n = cross(w_xv, w_xd);
            sin_theta = norm(n);
            if sin_theta > 1e-6
                n = n / sin_theta;
            else
                n = zeros(3,1);
            end

            % Signed angle
            cos_theta = dot(w_xv, w_xd);
            theta = atan2(sin_theta, cos_theta);

            obj.xdotbar = 0.4 * theta * n;
            obj.xdotbar = Saturate(obj.xdotbar, 0.3);

            robot.err_angle = theta;  % store heading error in robot model
        end

        function updateJacobian(obj, robot)
            % Jacobian maps vehicle angular velocities to heading change
            obj.J = [zeros(3,10) eye(3)]; % last 3 vehicle DOFs = angular velocities
        end

        function updateActivation(obj, robot)
            % Inequality task
            if isempty(robot.err_angle)
                theta = 0;
            else
                theta = robot.err_angle;
            end            
            obj.A = IncreasingBellShapedFunction(0.05, 0.15, zeros(3,1), eye(3), theta);
        end
    end
end
