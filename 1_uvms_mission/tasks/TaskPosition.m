classdef TaskPosition < Task
    properties
        gain = 0.7    % proportional gain to reference velocity
        error
    end
 
    methods
 
        function updateReference(obj, robot) % Compute desired reference velocity for the vehicle position task
            [v_ang, v_lin] = CartError(robot.wTgv , robot.wTv);% errore tra veicolo e goal_veicolo rispetto al mondo
            R = robot.vTw(1:3,1:3);
            obj.xdotbar = -obj.gain * R * v_lin;
            obj.xdotbar = Saturate(obj.xdotbar(1:3), 0.7);
            obj.error = norm(v_lin(1:2));

        end
 
        function updateJacobian(obj, robot)
            % for a task of cartesian position, J = I
            % Relationship between vehicle velocity (nu) and position rate in world
          
            % So J = [I3  0], dimension (3x6)
 
            J_vehicle = [-eye(3) zeros(3,3)]; 
            % The full system has 13 DOFs: [v_nu; q_dot]
            % So we append zeros for the manipulator part (7 joints)
            obj.J = [zeros(3,7) J_vehicle];
        end
 
        function updateActivation(obj, robot)
 
            obj.A = eye(3); % is an equality task = always active
        end
 
    end
end
 