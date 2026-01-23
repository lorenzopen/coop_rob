classdef TaskOrientation < Task
    properties
        error % Scalar for logging purposes
    end

    methods
        function updateReference(obj, robot)
            % Compute angular error vector (axis * angle) between Goal and Current
            [ang_err, ~] = CartError(robot.wTgv, robot.wTv);

            % Apply proportional gain (0.5) and saturate output (0.4)
            obj.xdotbar = Saturate(0.5 * ang_err, 0.4);

            % Store error norm for analysis
            obj.error = norm(ang_err);
        end

        function updateJacobian(obj, robot)
            % Extract rotation matrix (World to Vehicle)
            wRv = robot.wTv(1:3, 1:3);

            % Construct Jacobian mapping body angular vel to world angular vel
            % Structure: [Arm(7) | BaseLin(3) | BaseAng(3)]
            % We only map the last block using the rotation matrix
            obj.J = [zeros(3, 10), wRv];
        end

        function updateActivation(obj, ~)
            % Full activation on all 3 rotational axes
            obj.A = eye(3);
        end
    end
end