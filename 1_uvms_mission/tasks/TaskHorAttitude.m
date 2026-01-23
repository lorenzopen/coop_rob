classdef TaskHorAttitude < Task
    % Horizontal Attitude
    properties
    end
    methods
        function updateReference(obj, robot)
            % Vehicle z-axis
            w_kv = robot.wTv(1:3,3);
            % World z-axis
            w_kw = [0; 0; 1];
        
            n = cross(w_kw, w_kv);           % rotation axis (direction of misalignment)
            sin_theta = norm(n);             % magnitude of misalignment
            cos_theta = dot(w_kw, w_kv);     % alignment measure
            theta = atan2(sin_theta, cos_theta);
            
            % Calculate reference velocity (proportional to misalignment)
            obj.xdotbar = 1.5 * (0.1 - theta);
            obj.xdotbar = Saturate(obj.xdotbar, 0.5);
        end
        function updateJacobian(obj, robot)
            w_kv = robot.wTv(1:3,3);
            w_kw = [0; 0; 1];
            
            % Calculate cross product matrix for misalignment control
            n = cross(w_kv, w_kw);
            if norm(n) > 0
                n = n/norm(n);
            end
            
            % Jacobian maps vehicle angular velocities to misalignment reduction
            obj.J = n' * [zeros(3,7) zeros(3) eye(3)];
        end
        
        function updateActivation(obj, robot)
            % calculate theta
            w_kv = robot.wTv(1:3,3);
            w_kw = [0; 0; 1];
            theta = acos(dot(w_kv, w_kw));
            
            % Inequality task: theta < 0.2
            obj.A = IncreasingBellShapedFunction(0.1, 0.2, 0, 1, theta);
        end
    end
end 