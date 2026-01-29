classdef ObjectTask < Task
    % ObjectTask (Cooperative - Decentralized)
    % Calcola il rif di velocità per il frame oggetto e
    % lo Jacobiano per un SINGOLO braccio.
    
    properties
        gain = 1.0;
    end

    methods
        function obj = ObjectTask(robot_ID, taskID)
            obj.ID = robot_ID;
            obj.task_name = taskID;
        end

        function updateReference(obj, robot)
            % 'robot' single arm
         
            
            % cartesian error
            % wTog: Goal pose object (Global)
            % wTo: current pose object
            [v_ang, v_lin] = CartError(robot.wTog, robot.wTo);

            % Log 
            % robot.dist_to_goal = norm(v_lin);
            % robot.rot_to_goal = norm(v_ang);

            % desired vel (Reference)
            obj.xdotbar = obj.gain * [v_ang; v_lin];

            % 3. Saturation (Safety)
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.3); % Rad/s
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.3); % m/s
        end

        function updateJacobian(obj, robot)
            
            % 1. Jacobian(6x7)
            J_tool = robot.wJt;

            % 2.
            % evaluate r_toc: distance tool object frame World
            % wTt = pose tool, wTo = pose oggetto
            r_toc = robot.wTo(1:3, 4) - robot.wTt(1:3, 4);

            % skew r_toc
            S_r = skew(r_toc); 
            
            
            % omega_obj = omega_tool
            % v_obj = v_tool + omega_tool x r_toc = v_tool - r_toc x omega_tool
            %       = v_tool + S_r * omega_tool 
           
            
            wS_toc = [eye(3), zeros(3); 
                      -S_r,   eye(3)];

            % 3. Jacobiano proiettato sull'oggetto (6x7)
            
            % Questo Jacobiano lega le q_dot del singolo robot alla v_obj.
            obj.J = wS_toc * J_tool;
        end

        function updateActivation(obj, robot)
            % Attivazione sempre 
            obj.A = eye(6);
        end
    end
end