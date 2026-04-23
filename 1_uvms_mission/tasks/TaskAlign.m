classdef TaskAlign < Task   
    properties
        theta
        w_n
        w_d
        v_n
        v_d
        gain = 0.3
    end
    methods
        
        function updateReference(obj, robot)
            w_P = eye(3) - [0; 0; 1]*[0 0 1];
            [~, w_lin] = CartError(robot.wTg , robot.wTv);
            obj.w_d = w_P * w_lin;
            obj.v_d = robot.wTv(1:3,1:3)' * obj.w_d;
            v_i_v = [1; 0; 0];
            w_iv = robot.wTv(1:3,1:3) * v_i_v;
            % Protezione NaN per il landing
            if norm(obj.w_d) < 1e-5
               w_nd = w_iv; % Se siamo sopra al target, diciamo che siamo già allineati
            else
               w_nd = obj.w_d / norm(obj.w_d);
            end            
            obj.w_n = cross(w_iv,w_nd);
            sin_theta = norm(obj.w_n);
            cos_theta = w_iv' * w_nd;
            obj.theta = atan2(sin_theta, cos_theta);
            if sin_theta < 1e-5
                obj.w_n = [0; 0; 1];
            else
                obj.w_n = obj.w_n/sin_theta;
            end
            obj.v_n = robot.wTv(1:3,1:3)' * obj.w_n;          
            % I can do with rho (obj.theta * obj.v_n)
            obj.xdotbar = - obj.gain * obj.theta * obj.v_n;
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        
        function updateJacobian(obj, robot)
            epsilon = 1e-3; % Un valore piccolo per evitare la singolarità
            denominatore = norm(obj.v_d)^2 + epsilon;
            obj.J = (obj.v_n * obj.v_n' - (obj.theta/sin(obj.theta)) * skew([1;0;0]) * skew(obj.v_d/norm(obj.v_d)) * (eye(3) - (obj.v_n * obj.v_n'))) * [zeros(3,7), -(1/denominatore)*skew(obj.v_d), -eye(3)];
            %obj.J = (obj.v_n * obj.v_n' - (obj.theta/sin(obj.theta)) * skew([1;0;0]) * skew(obj.v_d/norm(obj.v_d)) * (eye(3) - (obj.v_n * obj.v_n'))) * [zeros(3,7), -(1/norm(obj.v_d)^2)*skew(obj.v_d), -eye(3)];
            %J_linear_veh = -(1/norm(obj.v_d)^2) * skew(obj.v_d);
            
            %obj.J = obj.v_n' * [zeros(3,7), J_linear_veh, -eye(3)];
        end
    
        function updateActivation(obj, robot)
            %obj.A = 1; % eye(3)
            obj.A = eye(3);
        end
    end
end