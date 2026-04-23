% classdef TaskHorizontalAttitude < Task   
%     properties
%         n
%         theta
%         % theta_star is the goal for the equivalent equality task
%         theta_star = 0
%         theta_full_activation = 0.1
%         gain = 0.2
%     end


%     methods
        

%         function updateReference(obj, robot)
%             % In this task our control variable x is the misalignment
%             % vector norm (theta) between k_v and k_w so it is a scalar
%             v_k_v = [0 0 1]';
%             v_k_w = robot.vTw(1:3,3);
%             rho = ReducedVersorLemma(v_k_v, v_k_w); % rot vector which makes k_w go to k_v
%             % Get axis angle representation equivalent (n*theta = rho)
%             obj.theta = norm(rho);
%             if obj.theta > 1e-6
%                 obj.n = rho / obj.theta;
%             else
%                 obj.n = [0, 0, 0]';
%             end
%             % Get reference rate by multiplying gain (0.2) by error between
%             % theta_star and theta
%             obj.xdotbar = - obj.gain * (obj.theta_star - obj.theta);
%             % limit the requested velocities...
%             obj.xdotbar = Saturate(obj.xdotbar, 0.2);
%         end
%         function updateJacobian(obj, robot)
%             obj.J = obj.n' * [zeros(3,10), eye(3)];
%         end
        
%         function updateActivation(obj, robot)

%             % update the task's inherent activation
%             obj.A = IncreasingBellShapedFunction(obj.theta_star, obj.theta_full_activation, 0, 1, obj.theta);
            
%         end
%     end
% end

classdef TaskHorizontalAttitude < Task
    % This task enforces the Horizontal Attitude constraint for a UVMS.
    % It is formulated as an inequality task that tries to align the
    % vehicle's z-axis with the global vertical direction [0;0;1].
 
    properties
        % No additional properties are required for this task
    end
   
    methods
        function updateReference(obj, robot)
            % Extract the vehicle's z-axis expressed in the world frame
            kv = robot.vTw(1:3,3);
 
            % Compute the component orthogonal to the vertical axis
            % rho measures how "far" the vehicle's z-axis is from vertical
            rho = norm(skew(kv) * [0; 0; 1]);
 
            % Projection of the z-axis onto the vertical direction
            h = dot(kv, [0 0 1]);
 
            % Compute tilt angle between the vehicle's z-axis and the world vertical
            theta = atan2(rho, h);
 
            % Control gain (lambda), determines correction speed
            Kp = 0.2;
 
            % Desired "velocity" output of the task:
            % we want to reduce theta → drive it back toward 0
            obj.xdotbar = -Kp * theta;
           
            % Safety saturation (disabled)
            % obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
       
        function updateJacobian(obj, robot)
            % Jacobian of the manipulator arm does not affect attitude
            J_arm = zeros(3, 7);
 
            % Vehicle angular velocity directly affects attitude
            % The vehicle Jacobian corresponds to the rotational part
            J_vehicle = [zeros(3, 3), eye(3)];
 
            % Compute the unit rotation axis that moves kv toward the vertical axis
            kv = robot.vTw(1:3,3);
            n = skew(kv) * [0; 0; 1];
 
            % Normalize it to obtain the rotation direction
            n = n / norm(n);
 
            % Full task Jacobian (row vector)
            obj.J = n' * [J_arm, J_vehicle];
        end
       
        function updateActivation(obj, robot)
            % Compute the same angle theta as in updateReference
            kv = robot.vTw(1:3,3);
            rho = norm(skew(kv) * [0; 0; 1]);
            h = dot(kv, [0 0 1]);
            theta = atan2(rho, h);
 
            % Activation function:
            % The task becomes active when the tilt angle exceeds 0.1 rad,
            % and fully active after 0.2 rad.
            obj.A = IncreasingBellShapedFunction(0.1, 0.2, 0, 1, theta);
        end
    end
end
 