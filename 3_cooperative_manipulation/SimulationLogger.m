classdef SimulationLogger < handle
    properties
        t            % time vector
        q            % joint positions
        qdot        % joint velocities
        a            % task activations (diagonal only)
        xdotbar_task % reference velocities for tasks (cell array)
        robot        % robot model
        action_set     % set of actions
        xdot            %End Effector velocity
        n
    end

    methods
        function obj = SimulationLogger(maxLoops, robot)
            obj.robot = robot;

            obj.t = zeros(1, maxLoops);
            obj.q = zeros(7, maxLoops);
            obj.qdot = zeros(7, maxLoops);
            %Optional: plot the end-effector velocities
            obj.xdot=zeros(6,maxLoops);
            
        end

        function update(obj, t, loop)
            % Store robot state
            obj.t(loop) = t;
            obj.q(:, loop) = obj.robot.q;
            obj.qdot(:, loop) = obj.robot.qdot;


        end
        function plotAll(obj)
                % Example plotting for robot state
                figure;
                subplot(2,1,1);
                plot(obj.t, obj.q,'.-','LineWidth', 2);
                legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7');
                title('Joint positions');
                subplot(2,1,2);
                plot(obj.t, obj.qdot,'.-', 'LineWidth', 2);
                legend('qd_1','qd_2','qd_3','qd_4','qd_5','qd_6','qd_7');
                title('Joint velocities');
                %Optional: Plot the end effector velocities
                % figure;
                % plot(obj.t, obj.xdot,'.-', 'LineWidth', 2);
                % legend('wx','wy','wz','vx','vy','vz')
                % grid on
                % title('End-effector velocities');    
            end

    end
end