classdef SimulationLogger < handle
    properties
        t            % time vector
        q            % joint positions
        q_dot        % joint velocities
        eta          % vehicle pose
        v_nu         % vehicle velocity
        a            % task activations (diagonal only)
        xdotbar_task % reference velocities for tasks (cell array)
        robot        % robot model
        task_set     % set of tasks
        
        % Nuove proprietà per i plot richiesti
        tool_vel     % Velocità tool [6 x loops] (1:3 Angolare, 4:6 Lineare)
        mission_phase % Fase missione [1 x loops]
        altitude     % Altitude storage [1 x loops]
        tilt_angle   % Misalignment angle (rho) for Horizontal Attitude [1 x loops]
    end

    methods
        % --- Costruttore ---
        function obj = SimulationLogger(maxLoops, robotModel, task_set)
            obj.robot = robotModel;
            obj.task_set = task_set;

            obj.t = zeros(1, maxLoops);
            obj.q = zeros(7, maxLoops);
            obj.q_dot = zeros(7, maxLoops);
            obj.eta = zeros(6, maxLoops);
            obj.v_nu = zeros(6, maxLoops);
            
            % Inizializzazione nuove variabili
            obj.tool_vel = zeros(6, maxLoops);
            obj.mission_phase = zeros(1, maxLoops);
            obj.altitude = zeros(1, maxLoops);
            obj.tilt_angle = zeros(1, maxLoops);

            maxDiagSize = max(cellfun(@(t) size(t.A,1), task_set));
            obj.a = zeros(maxDiagSize, maxLoops, length(task_set));

            obj.xdotbar_task = cell(length(task_set), maxLoops);
        end

        % --- Funzione Update ---
        function update(obj, t, loop, missionPhase)
            % Store robot state
            obj.t(loop) = t;
            obj.q(:, loop) = obj.robot.q;
            obj.q_dot(:, loop) = obj.robot.q_dot;
            obj.eta(:, loop) = obj.robot.eta;
            obj.v_nu(:, loop) = obj.robot.v_nu;
            
            % Store Mission Phase (gestione argomento opzionale)
            if nargin < 4
                missionPhase = 1; 
            end
            obj.mission_phase(loop) = missionPhase;

            % 1. FIX: Controllo isempty sulla Altitude
            if isempty(obj.robot.altitude)
                obj.altitude(loop) = 0.0; % Fallback
            else
                obj.altitude(loop) = obj.robot.altitude;
            end
            
            % 2. Calcolo Tilt Angle (Horizontal Attitude)
            % Estraiamo asse Z veicolo nel frame mondo
            % vTw è disponibile nel robot model aggiornato
            kv = obj.robot.vTw(1:3,3); 
            % Calcolo angolo rispetto alla verticale [0;0;1]
            % rho = norm(cross(kv, [0;0;1])); % componente ortogonale
            % h = dot(kv, [0;0;1]);           % componente parallela
            % theta = atan2(rho, h);
            % O più semplicemente usando acos (dato che sono versori)
            % theta = acos(dot(kv, [0;0;1]));
            
            % Replichiamo esattamente la logica della TaskHorizontalAttitude per coerenza
            rho_val = norm(cross(kv, [0;0;1]));
            h_val = dot(kv, [0;0;1]);
            obj.tilt_angle(loop) = atan2(rho_val, h_val);

            % 3. Calcolo Velocità Tool (J_tool * [q_dot; v_nu])
            ydot = [obj.robot.q_dot; obj.robot.v_nu];
            for i = 1:length(obj.task_set)
                if isa(obj.task_set{i}, 'TaskTool')
                     obj.tool_vel(:, loop) = obj.task_set{i}.J * ydot;
                end
            end
            
            % Store task activations e reference velocities
            for i = 1:length(obj.task_set)
                if ~isempty(obj.task_set{i}.A)
                    diagA = diag(obj.task_set{i}.A);
                    obj.a(1:length(diagA), loop, i) = diagA;
                end
                obj.xdotbar_task{i, loop} = obj.task_set{i}.xdotbar;
            end
        end

        % --- Funzione PlotAll ---
        function plotAll(obj)
            % Trova i cambi di fase per le linee verticali
            phase_changes = find(diff(obj.mission_phase) ~= 0);

            %% 1. Joint Positions & Velocities
            figure('Name', 'Joint Space Analysis');
            subplot(2,1,1);
            plot(obj.t, obj.q, 'LineWidth', 1);
            title('Arm Joint Positions (q)'); legend('q_1','q_2','q_3','q_4','q_5','q_6','q_7','Location','eastoutside'); grid on;
            for pc = phase_changes, xline(obj.t(pc), '--r', 'HandleVisibility', 'off'); end

            subplot(2,1,2);
            plot(obj.t, obj.q_dot, 'LineWidth', 1);
            title('Arm Joint Velocities (q_{dot})'); legend('qd_1','qd_2','qd_3','qd_4','qd_5','qd_6','qd_7','Location','eastoutside'); grid on; ylabel('rad/s');
            for pc = phase_changes, xline(obj.t(pc), '--r', 'HandleVisibility', 'off'); end

            %% 2. Velocità Cartesiane Complete
            figure('Name', 'Cartesian Velocities: Vehicle & Tool');
            
            subplot(4,1,1);
            plot(obj.t, obj.v_nu(1:3, :), 'LineWidth', 1); 
            title('Vehicle Linear Velocity'); legend('v_x','v_y','v_z','Location','best'); grid on; ylabel('m/s');
            for pc = phase_changes, xline(obj.t(pc), '--r', 'HandleVisibility', 'off'); end
            
            subplot(4,1,2);
            plot(obj.t, obj.v_nu(4:6, :), 'LineWidth', 1); 
            title('Vehicle Angular Velocity'); legend('w_x','w_y','w_z','Location','best'); grid on; ylabel('rad/s');
            for pc = phase_changes, xline(obj.t(pc), '--r', 'HandleVisibility', 'off'); end

            subplot(4,1,3);
            plot(obj.t, obj.tool_vel(4:6, :), 'LineWidth', 1); 
            title('Tool Linear Velocity'); legend('vt_x','vt_y','vt_z','Location','best'); grid on; ylabel('m/s');
            for pc = phase_changes, xline(obj.t(pc), '--r', 'HandleVisibility', 'off'); end

            subplot(4,1,4);
            plot(obj.t, obj.tool_vel(1:3, :), 'LineWidth', 1); 
            title('Tool Angular Velocity'); legend('wt_x','wt_y','wt_z','Location','best'); grid on; ylabel('rad/s');
            for pc = phase_changes, xline(obj.t(pc), '--r', 'HandleVisibility', 'off'); end
            xlabel('Time [s]');

            %% 3. Analisi Task Altitude (Safety Hard)
            idx_alt = find(cellfun(@(x) isa(x, 'TaskAltitude'), obj.task_set));
            if ~isempty(idx_alt)
                figure('Name', 'Altitude Task Analysis');
                subplot(2,1,1);
                plot(obj.t, obj.altitude, 'b', 'LineWidth', 1.5); hold on;
                yline(3.0, '--r', 'Min (3.0m)', 'LineWidth', 1.2); 
                yline(2.0, '--g', 'Start (2.0m)', 'LineWidth', 1.2);
                title('Vehicle Altitude'); ylabel('m'); grid on;
                for pc = phase_changes, xline(obj.t(pc), '--k', 'HandleVisibility', 'off'); end
                
                subplot(2,1,2);
                act_alt = squeeze(obj.a(1, :, idx_alt)); 
                plot(obj.t, act_alt, 'm', 'LineWidth', 1.5);
                title('Activation Function'); ylabel('A [0-1]'); grid on;
                ylim([-0.1 1.1]);
                for pc = phase_changes, xline(obj.t(pc), '--k', 'HandleVisibility', 'off'); end
            end

            %% 4. Analisi Task Horizontal Attitude (Safety Soft)
            idx_horz = find(cellfun(@(x) isa(x, 'TaskHorizontalAttitude'), obj.task_set));
            if ~isempty(idx_horz)
                figure('Name', 'Horizontal Attitude Analysis');
                
                subplot(2,1,1);
                plot(obj.t, obj.tilt_angle, 'b', 'LineWidth', 1.5); hold on;
                yline(0.1, '--g', 'Soft Threshold (0.1 rad)', 'LineWidth', 1.2);
                yline(0.2, '--r', 'Max Tilt (0.2 rad)', 'LineWidth', 1.2);
                title('Vehicle Tilt Angle (\theta)'); ylabel('rad'); grid on;
                for pc = phase_changes, xline(obj.t(pc), '--k', 'HandleVisibility', 'off'); end
                
                subplot(2,1,2);
                act_horz = squeeze(obj.a(1, :, idx_horz));
                plot(obj.t, act_horz, 'm', 'LineWidth', 1.5);
                title('Horizontal Attitude Activation'); ylabel('A [0-1]'); grid on;
                ylim([-0.1 1.1]);
                for pc = phase_changes, xline(obj.t(pc), '--k', 'HandleVisibility', 'off'); end
            end
            
            %% 5. Panoramica Tutte le Attivazioni
             figure('Name', 'All Activations');
             n = length(obj.task_set);
             for i=1:n
                 subplot(n,1,i);
                 plot(obj.t, squeeze(obj.a(1,:,i)), 'LineWidth', 1);
                 tName = class(obj.task_set{i});
                 title(tName, 'Interpreter', 'none'); 
                 grid on; ylim([-0.1 1.1]);
                 if i == n, xlabel('Time [s]'); end
             end
        end
    end
end