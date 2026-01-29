classdef SimulationLogger < handle
    properties
        t            % time vector
        ql           % joint positions (Left)
        qdotl        % joint velocities (Left)
        qr           % joint positions (Right)
        qdotr        % joint velocities (Right)
        
        robot        % robot model (bm_sim)
        action_mng   % Action Manager reference
        
        tasks_ref    % global task list
        n_tasks      % number of tasks
        
        xdotbar_task % reference velocities (cell array history)
        a_task       % task activations data (cell array history)
        
        action_idx_log % History of action indices
        action_names   % List of action names
        n_actions      % Total number of distinct actions

        tool_pos_L
        tool_pos_R

        object_twist_actual_L   % 6xN actual object twist
        object_twist_actual_R   % 6xN actual object twist

        object_xdotbar_L        % 6xN desired object twist for left arm
        object_xdotbar_R        % 6xN desired object twist for right arm
        
        % NEW: Object Pose History
        obj_pos      % 3xN Object Position (World Frame)
        obj_rpy      % 3xN Object Orientation (RPY)
        
        data_len     % valid data length
    end

    methods

        function obj = SimulationLogger(maxLoops, robotModel, actionManager)
            obj.robot = robotModel;
            obj.action_mng = actionManager;
            
            % Estrae i dati dall'ActionManager
            obj.tasks_ref = actionManager.unifiedList; 
            obj.n_tasks = length(obj.tasks_ref);
            obj.action_names = actionManager.actionsName;
            obj.n_actions = length(obj.action_names);
            
            obj.data_len = 0;
            
            % Allocazione memoria
            obj.t = zeros(1, maxLoops);
            obj.ql = zeros(7, maxLoops);
            obj.qdotl = zeros(7, maxLoops);
            obj.qr = zeros(7, maxLoops);
            obj.qdotr = zeros(7, maxLoops);
            
            obj.tool_pos_L = zeros(3, maxLoops);
            obj.tool_pos_R = zeros(3, maxLoops);

            obj.object_twist_actual_L = nan(6, maxLoops);
            obj.object_twist_actual_R = nan(6, maxLoops);
            
            obj.object_xdotbar_L = nan(6, maxLoops);
            obj.object_xdotbar_R = nan(6, maxLoops);
            
            obj.xdotbar_task = cell(obj.n_tasks, maxLoops);
            obj.a_task = cell(obj.n_tasks, maxLoops);
            
            % NEW: Object Pose Init
            obj.obj_pos = nan(3, maxLoops);
            obj.obj_rpy = nan(3, maxLoops);

            obj.action_idx_log = ones(1, maxLoops); 
        end
        

        function update(obj, t, loop)
            % Basic state logger
            obj.t(loop) = t;
            obj.ql(:, loop) = obj.robot.left_arm.q;
            obj.qdotl(:, loop) = obj.robot.left_arm.qdot;
            obj.qr(:, loop) = obj.robot.right_arm.q;
            obj.qdotr(:, loop) = obj.robot.right_arm.qdot;
            
            % Store Tool Positions
            if ~isempty(obj.robot.left_arm.wTt)
                obj.tool_pos_L(:, loop) = obj.robot.left_arm.wTt(1:3, 4);
            end
            if ~isempty(obj.robot.right_arm.wTt)
                obj.tool_pos_R(:, loop) = obj.robot.right_arm.wTt(1:3, 4);
            end
            
            % Store Object Pose (Using Left Arm knowledge of wTo)
            wTo_curr = obj.robot.left_arm.wTo;
            if ~isempty(wTo_curr)
                obj.obj_pos(:, loop) = wTo_curr(1:3, 4);
                try
                    obj.obj_rpy(:, loop) = RotMatrix2RPY(wTo_curr(1:3, 1:3));
                catch
                    obj.obj_rpy(:, loop) = [0;0;0]; 
                end
            end
            
            % Log Action Index
            obj.action_idx_log(loop) = obj.action_mng.currentAction;
            
            % Log Tasks references / activations
            for i = 1:obj.n_tasks
                obj.xdotbar_task{i, loop} = obj.tasks_ref{i}.xdotbar;
                obj.a_task{i, loop} = diag(obj.tasks_ref{i}.A);
            end
            
            % --- Automatic Object Data Extraction ---
            qdot_full = [obj.robot.left_arm.qdot; obj.robot.right_arm.qdot];

            % Left Object Task
            task_obj_L = obj.action_mng.getTaskByName('OBJECT_MOTION_L');
            if ~isempty(task_obj_L)
                obj.object_xdotbar_L(:, loop) = task_obj_L.xdotbar;
                if ~isempty(task_obj_L.J)
                   obj.object_twist_actual_L(:, loop) = task_obj_L.J * qdot_full;
                end
            end

            % Right Object Task
            task_obj_R = obj.action_mng.getTaskByName('OBJECT_MOTION_R');
            if ~isempty(task_obj_R)
                obj.object_xdotbar_R(:, loop) = task_obj_R.xdotbar;
                if ~isempty(task_obj_R.J)
                   obj.object_twist_actual_R(:, loop) = task_obj_R.J * qdot_full;
                end
            end
            
            % Update data length
            obj.data_len = loop;
        end


        % =================================================================
        % HELPER: ADD PHASE LINES (Tratteggiate Verticali)
        % =================================================================
        function add_phase_lines(obj, h_ax)
            idx_valid = 1:obj.data_len;
            phases = obj.action_idx_log(idx_valid);
            time = obj.t(idx_valid);
            
            idx_ph2 = find(phases == 2, 1, 'first');
            idx_ph3 = find(phases == 3, 1, 'first');
            
            if ~isempty(idx_ph2)
                t_ph2 = time(idx_ph2);
                xline(h_ax, t_ph2, 'k--', 'Start Manip', ...
                    'LineWidth', 1.2, 'LabelVerticalAlignment', 'bottom', ...
                    'FontSize', 10, 'LabelHorizontalAlignment', 'left');
            end
            
            if ~isempty(idx_ph3)
                t_ph3 = time(idx_ph3);
                xline(h_ax, t_ph3, 'k--', 'Stop Manip', ...
                    'LineWidth', 1.2, 'LabelVerticalAlignment', 'bottom', ...
                    'FontSize', 10, 'LabelHorizontalAlignment', 'right');
            end
        end


        % =================================================================
        % PLOT: JOINT LIMITS ACTIVATION (RICHIESTA UTENTE)
        % =================================================================
        function plotJointLimitActivations(obj)
            idx_valid = 1:obj.data_len;
            time = obj.t(idx_valid);
            
            act_JL_L = zeros(1, obj.data_len);
            act_JL_R = zeros(1, obj.data_len);
            
            % Cerca i task JL_Safe e JR_Safe (nomi dal main.m)
            idx_L = find(cellfun(@(x) strcmp(x.task_name, 'JL_Safe'), obj.tasks_ref));
            idx_R = find(cellfun(@(x) strcmp(x.task_name, 'JR_Safe'), obj.tasks_ref));
            
            if ~isempty(idx_L)
                raw = obj.a_task(idx_L, idx_valid);
                % Estraiamo il massimo livello di attivazione tra i giunti per ogni istante
                for k=1:length(raw)
                    if ~isempty(raw{k})
                        act_JL_L(k) = max(raw{k}); 
                    end
                end
            end
            
            if ~isempty(idx_R)
                raw = obj.a_task(idx_R, idx_valid);
                for k=1:length(raw)
                    if ~isempty(raw{k})
                        act_JL_R(k) = max(raw{k});
                    end
                end
            end
            
            figure('Name', 'Joint Limits Activation', 'Color', 'w', 'Position', [100 600 800 500]);
            ax = axes;
            plot(time, act_JL_L, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Left Arm (Max Joint Act)'); hold on;
            plot(time, act_JL_R, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Right Arm (Max Joint Act)');
            grid on;
            ylabel('Activation \alpha');
            xlabel('Time [s]');
            title('Joint Limits Task Activation (Max per Arm)');
            legend('Location', 'best');
            ylim([-0.1 1.1]);
            
            % Aggiungi le linee di fase
            obj.add_phase_lines(ax);
        end


        % =================================================================
        % PLOT 4: Q4 ANALYSIS (Esercizio 2)
        % =================================================================
        function plotQ4Analysis(obj)
            idx_valid = 1:obj.data_len;
            time = obj.t(idx_valid);
            phases = obj.action_idx_log(idx_valid);
            
            idx_manip_start = find(phases == 2, 1, 'first');
            if isempty(idx_manip_start), return; end
            
            % 1. Distanza
            diff_vec = obj.tool_pos_L(:, idx_valid) - obj.tool_pos_R(:, idx_valid);
            dist_norm = sqrt(sum(diff_vec.^2, 1));
            
            % 2. Velocità
            vel_ref_obj = vecnorm(obj.object_xdotbar_L(:, idx_valid), 2, 1);
            vel_real_L = vecnorm(obj.object_twist_actual_L(:, idx_valid), 2, 1);
            vel_real_R = vecnorm(obj.object_twist_actual_R(:, idx_valid), 2, 1);
            
            vel_ref_obj(1:idx_manip_start-1) = NaN;
            vel_real_L(1:idx_manip_start-1) = NaN;
            vel_real_R(1:idx_manip_start-1) = NaN;

            % 3. Safety Activations (JL & Alt)
            act_JL_L = zeros(1, obj.data_len);
            act_Alt_L = zeros(1, obj.data_len);
            
            % Recupero attivazioni
            for k=1:obj.n_tasks
                tname = obj.tasks_ref{k}.task_name;
                raw_act = obj.a_task(k, idx_valid);
                act_vec = zeros(1, obj.data_len);
                for j=1:length(raw_act), if ~isempty(raw_act{j}), act_vec(j) = max(raw_act{j}); end, end
                
                if contains(tname, 'JL_Safe'), act_JL_L = act_vec; end
                if contains(tname, 'TOOL_ALT') && contains(tname, 'L'), act_Alt_L = act_vec; end
            end
            
            figure('Name', 'Q4 Analysis: Constraint & Coordination', 'Color', 'w', 'Position', [100 50 800 900]);
            
            ax1 = subplot(3,1,1);
            plot(time, dist_norm, 'k-', 'LineWidth', 1.5);
            ylabel('Tool Dist [m]'); title('1. Rigid Constraint Verification'); grid on;
            ylim([min(dist_norm)*0.99, max(dist_norm)*1.01]);
            obj.add_phase_lines(ax1);
            
            ax2 = subplot(3,1,2);
            plot(time, vel_ref_obj, 'g--', 'LineWidth', 2); hold on;
            plot(time, vel_real_L, 'b-'); plot(time, vel_real_R, 'r:');
            ylabel('Vel Norm [m/s]'); title('2. Bimanual Coordination'); legend('Ref','Real L','Real R'); grid on;
            obj.add_phase_lines(ax2);
            
            ax3 = subplot(3,1,3);
            plot(time, act_JL_L, 'b-', 'DisplayName', 'Joint Limits (Max)'); hold on;
            plot(time, act_Alt_L, 'm-', 'DisplayName', 'Altitude');
            ylabel('Act \alpha'); xlabel('Time [s]'); title('3. Stress Analysis'); legend; grid on; ylim([-0.1 1.1]);
            obj.add_phase_lines(ax3);
            
            xlim([time(idx_manip_start)-1, time(end)]);
        end


        % =================================================================
        % PLOT 1: SUMMARY STYLE
        % =================================================================
        function plotSummaryExerciseStyle(obj)
            idx_valid = 1:obj.data_len;
            time = obj.t(idx_valid);
            phases = obj.action_idx_log(idx_valid);
            idx_start_manip = find(phases == 2, 1, 'first');

            act_alt_L = zeros(1, obj.data_len);
            for k=1:obj.n_tasks
                if contains(obj.tasks_ref{k}.task_name, 'TOOL_ALT_TASK_L')
                    raw_act = obj.a_task(k, idx_valid);
                    for j=1:length(raw_act), if ~isempty(raw_act{j}), act_alt_L(j) = raw_act{j}(1); end, end
                end
            end

            norm_vel_LT = zeros(1, obj.data_len);
            norm_vel_RT = zeros(1, obj.data_len);
            idx_LT = find(cellfun(@(x) strcmp(x.task_name, 'LT'), obj.tasks_ref));
            idx_RT = find(cellfun(@(x) strcmp(x.task_name, 'RT'), obj.tasks_ref));
            if ~isempty(idx_LT)
                raw = obj.xdotbar_task(idx_LT, idx_valid);
                norm_vel_LT = cellfun(@(x) norm(x), raw);
            end
            if ~isempty(idx_RT)
                raw = obj.xdotbar_task(idx_RT, idx_valid);
                norm_vel_RT = cellfun(@(x) norm(x), raw);
            end
            if ~isempty(idx_start_manip)
                norm_vel_LT(idx_start_manip:end) = NaN;
                norm_vel_RT(idx_start_manip:end) = NaN;
            end

            err_obj_L = vecnorm(obj.object_xdotbar_L(:, idx_valid), 2, 1);
            
            figure('Name', 'Bimanual Mission Analysis', 'Color', 'w', 'Position', [50 50 900 800]);
            
            ax1 = subplot(3,1,1);
            yyaxis left; plot(time, obj.tool_pos_L(3, idx_valid), 'b-'); hold on; plot(time, obj.tool_pos_R(3, idx_valid), 'c-');
            ylabel('Tool Alt [m]'); yline(0.15, 'r--'); grid on;
            yyaxis right; plot(time, act_alt_L, 'r-'); ylabel('Act \alpha'); ylim([-0.1 1.1]);
            title('Safety: Altitude Regulation'); obj.add_phase_lines(ax1);

            ax2 = subplot(3,1,2);
            plot(time, norm_vel_LT, 'b-'); hold on; plot(time, norm_vel_RT, 'r-');
            ylabel('Appr Vel [m/s]'); title('Phase 1: Approach'); obj.add_phase_lines(ax2);
            
            ax3 = subplot(3,1,3);
            plot(time, err_obj_L, 'm-'); grid on; xlabel('Time [s]'); ylabel('Obj Ref');
            title('Phase 2: Object Regulation'); obj.add_phase_lines(ax3);
            if ~isempty(idx_start_manip), xlim([max(0, time(idx_start_manip)-1), time(end)]); end
        end

        
        % =================================================================
        % PLOT 2: OBJECT MOTION
        % =================================================================
        function plotObjectMotion(obj)
            idx_valid = 1:obj.data_len;
            time = obj.t(idx_valid);
            pos = obj.obj_pos(:, idx_valid);
            rpy_deg = rad2deg(obj.obj_rpy(:, idx_valid));

            figure('Name', 'Object Motion', 'Color', 'w', 'Position', [600 50 800 600]);
            
            ax1 = subplot(2,1,1);
            plot(time, pos(1,:), 'r-', 'DisplayName', 'X'); hold on;
            plot(time, pos(2,:), 'g-', 'DisplayName', 'Y');
            plot(time, pos(3,:), 'b-', 'DisplayName', 'Z');
            grid on; ylabel('Pos [m]'); title('Object Position'); legend('Location','best');
            obj.add_phase_lines(ax1);
            
            ax2 = subplot(2,1,2);
            plot(time, rpy_deg(1,:), 'r--', 'DisplayName', 'R'); hold on;
            plot(time, rpy_deg(2,:), 'g--', 'DisplayName', 'P');
            plot(time, rpy_deg(3,:), 'b--', 'DisplayName', 'Y');
            grid on; ylabel('Ori [deg]'); xlabel('Time [s]'); title('Object Orientation'); legend('Location','best');
            obj.add_phase_lines(ax2);
        end


        % =================================================================
        % MAIN PLOTTER
        % =================================================================
        function plotAll(obj, ~, task_selectors)
            obj.plotSummaryExerciseStyle();
            obj.plotObjectMotion();
            obj.plotJointLimitActivations(); % <--- NEW PLOT REQUESTED
            obj.plotQ4Analysis();            % <--- Q4 ANALYSIS PLOT
            
            obj.plotToolDistance();
            obj.plotObjectActualReal();

            if nargin < 3 || isempty(task_selectors), return; end
            
            if ischar(task_selectors) || isstring(task_selectors) || iscell(task_selectors)
                if ischar(task_selectors) || isstring(task_selectors), names_to_find = cellstr(task_selectors);
                else, names_to_find = task_selectors; end
                task_indices = [];
                for k = 1:length(names_to_find)
                    for i = 1:obj.n_tasks
                        if strcmpi(obj.tasks_ref{i}.task_name, names_to_find{k}), task_indices(end+1) = i; break; end
                    end
                end
            elseif isnumeric(task_selectors), task_indices = task_selectors;
            else, task_indices = []; end
            
            valid_range = 1:obj.data_len;
            time_vec = obj.t(valid_range);
            
            for i = 1:length(task_indices)
                idx = task_indices(i);
                t_name = char(obj.tasks_ref{idx}.task_name);
                figure('Name', t_name, 'Color', 'w');
                raw_ref = obj.xdotbar_task(idx, valid_range);
                raw_act = obj.a_task(idx, valid_range);
                if any(~cellfun(@isempty, raw_ref))
                    data_ref = cell2mat(raw_ref(~cellfun(@isempty, raw_ref)));
                    data_act = cell2mat(raw_act(~cellfun(@isempty, raw_act)));
                    t_plot = time_vec(~cellfun(@isempty, raw_ref));
                    
                    ax1 = subplot(2,1,1); plot(t_plot, data_ref'); grid on; title([t_name ' Ref']); obj.add_phase_lines(ax1);
                    ax2 = subplot(2,1,2); plot(t_plot, data_act'); grid on; title([t_name ' Act']); ylim([-0.1 1.1]); obj.add_phase_lines(ax2);
                end
            end
        end

        % Helpers
        function plotToolDistance(obj)
            valid_range = 1:obj.data_len;
            diff_vec = obj.tool_pos_L(:, valid_range) - obj.tool_pos_R(:, valid_range);
            dist_norm = sqrt(sum(diff_vec.^2, 1));
            figure(301); clf; set(gcf, 'Name', 'Tool Dist', 'Color', 'w'); ax=axes;
            plot(obj.t(valid_range), dist_norm, 'LineWidth', 2); grid on; title('Tool Dist'); obj.add_phase_lines(ax);
        end

        function plotObjectActualReal(obj)
            valid_range = 1:obj.data_len;
            time_vec = obj.t(valid_range);
            refL = vecnorm(obj.object_xdotbar_L(:, valid_range));
            actL = vecnorm(obj.object_twist_actual_L(:, valid_range));
            figure(305); clf; set(gcf, 'Name', 'Obj Vel', 'Color', 'w'); ax=axes;
            plot(time_vec, refL, 'b--'); hold on; plot(time_vec, actL, 'b-');
            title('Obj Vel'); legend('Ref', 'Act'); grid on; obj.add_phase_lines(ax);
        end
        
        function expand_ylim(~, h_ax), end % Deprecated
        function [legends, y_label_unit] = generate_legends(~, dims, name, type), legends = {}; y_label_unit = ''; end
        function format_plot(~, h_title, h_legend, h_ax, y_unit), set(h_title, 'Interpreter', 'latex'); end
    end
end