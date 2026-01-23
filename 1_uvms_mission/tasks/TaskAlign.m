classdef TaskAlign < Task
    methods
        function updateReference(obj, robot)
            % Get current vehicle heading (X-axis)
            u_curr = robot.wTv(1:3, 1);
 
            % Get target direction (vehicle to nodule, projected on XY plane)
            pos_nodule = robot.wTg(1:3, 4);
            vec_des = pos_nodule - robot.wTv(1:3, 4);
            vec_des(3) = 0;
 
            % Normalize desired vector (handle singularity)
            dist = norm(vec_des);
            if dist > 1e-3
                u_des = vec_des / dist;
            else
                u_des = u_curr;
            end
 
            % Compute orientation error (angle between current and desired)
            rot_vector = cross(u_des, u_curr);
            sin_val = norm(rot_vector);
            cos_val = dot(u_des, u_curr);
            err_angle = atan2(sin_val, cos_val);
 
            % Proportional control law
            obj.xdotbar = Saturate(1.0 * err_angle, 0.5);

            robot.err_angle = err_angle;  % save alignment error for logging
        end
 
        function updateJacobian(obj, robot)
            % Recompute vectors
            u_curr = robot.wTv(1:3, 1);
            pos_nodule = robot.wTg(1:3, 4);
            vec_des = pos_nodule - robot.wTv(1:3, 4);
            vec_des(3) = 0;
            if norm(vec_des) > 1e-3
                u_des = vec_des / norm(vec_des);
            else
                u_des = u_curr;
            end
 
            % Compute rotation axis
            axis_n = cross(u_curr, u_des);
            if norm(axis_n) > 0
                axis_n = axis_n / norm(axis_n);
            end
 
            % Jacobian: project vehicle angular velocity onto error axis
            obj.J = axis_n' * [zeros(3,7), zeros(3,3), eye(3)];
        end
 
        function updateActivation(obj, robot)
            % Always active (equality task)
            obj.A = 1.0;
        end
    end
end