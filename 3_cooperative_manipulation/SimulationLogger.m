classdef SimulationLogger < handle
    properties
        t               % time vector
        
        % Dati Giunti
        ql, qr          % posizioni
        qdotl, qdotr    % velocità reali (Coop)
        
        % Dati Cartesiani
        xdot_L_nc, xdot_L_act      
        xdot_R_nc, xdot_R_act      
        
        % Dati Task e Attivazioni
        xdotbar_task    
        a_task          
        tasks_ref       
        n_tasks         
        
        % Mission Phase
        action_idx_log  
        action_names    
        
        % Posizioni Tool
        tool_pos_L      
        tool_pos_R      
        
        robot           % riferimento a coop_system
        data_len        % valid data length
    end

    methods
        function obj = SimulationLogger(maxLoops, coop_system, tasks_L, tasks_R)
            obj.robot = coop_system;
            obj.tasks_ref = [tasks_L, tasks_R]; 
            obj.n_tasks = length(obj.tasks_ref);
            
            obj.action_names = {'Grasping', 'Cooperative Manipulation', 'Stop'};
            
            obj.data_len = 0;
            
            % Pre-allocazione
            obj.t = zeros(1, maxLoops);
            obj.ql = zeros(7, maxLoops); obj.qr = zeros(7, maxLoops);
            obj.qdotl = zeros(7, maxLoops); obj.qdotr = zeros(7, maxLoops);
            
            % Allocazione Cartesiana (6 DOF)
            obj.xdot_L_nc = zeros(6, maxLoops);
            obj.xdot_L_act = zeros(6, maxLoops);
            obj.xdot_R_nc = zeros(6, maxLoops);
            obj.xdot_R_act = zeros(6, maxLoops);
            
            obj.tool_pos_L = zeros(3, maxLoops);
            obj.tool_pos_R = zeros(3, maxLoops);
            
            obj.xdotbar_task = cell(obj.n_tasks, maxLoops);
            obj.a_task = cell(obj.n_tasks, maxLoops);
            obj.action_idx_log = ones(1, maxLoops); 
        end
        
        function update(obj, t, loop, mission_phase, ql_nc, qr_nc)
            % --- 1. Aggiornamento Base ---
            obj.t(loop) = t;
            obj.ql(:, loop) = obj.robot.left_arm.q;
            obj.qr(:, loop) = obj.robot.right_arm.q;
            obj.qdotl(:, loop) = obj.robot.left_arm.qdot;
            obj.qdotr(:, loop) = obj.robot.right_arm.qdot;
            
            % --- 2. Calcolo Velocità Cartesiane ---
            J_L = obj.robot.left_arm.wJo;
            J_R = obj.robot.right_arm.wJo;
            
            % Fallback se wJo non è ancora inizializzato (Fase 1)
            if isempty(J_L), J_L = obj.robot.left_arm.wJt; end
            if isempty(J_R), J_R = obj.robot.right_arm.wJt; end
            
            if ~isempty(ql_nc), obj.xdot_L_nc(:, loop) = J_L * ql_nc; end
            if ~isempty(qr_nc), obj.xdot_R_nc(:, loop) = J_R * qr_nc; end
            
            obj.xdot_L_act(:, loop) = J_L * obj.robot.left_arm.qdot;
            obj.xdot_R_act(:, loop) = J_R * obj.robot.right_arm.qdot;
            
            % --- 3. Posizioni Tool ---
            obj.tool_pos_L(:, loop) = obj.robot.left_arm.wTt(1:3, 4);
            obj.tool_pos_R(:, loop) = obj.robot.right_arm.wTt(1:3, 4);
        
            % --- 4. Task Data ---
            for i = 1:obj.n_tasks
                ct = obj.tasks_ref{i};
                obj.xdotbar_task{i, loop} = ct.xdotbar;
                if isprop(ct, 'A')
                    if ismatrix(ct.A) && min(size(ct.A))>1 
                        obj.a_task{i, loop} = diag(ct.A);
                    else
                        obj.a_task{i, loop} = ct.A; 
                    end
                else
                    obj.a_task{i, loop} = 1; 
                end
            end
        
            if isempty(mission_phase), mission_phase = 1; end
            obj.action_idx_log(loop) = mission_phase;
            obj.data_len = loop;
        end
        
        % =========================================================
        % === 1. SAFETY TASKS PLOT (ALTITUDE & JOINT LIMITS) ===
        % =========================================================
        function plotSafetyTasks(obj)
            if obj.data_len == 0, warning('No data to plot'); return; end
            range = 1:obj.data_len;
            t_vec = obj.t(range);
            
            idx_alt_L = obj.findTaskIdx('ALT_L');
            idx_alt_R = obj.findTaskIdx('ALT_R');
            idx_jl_L  = obj.findTaskIdx('JL_L');
            idx_jl_R  = obj.findTaskIdx('JL_R');
            
            figure('Name', 'Safety Tasks Analysis', 'Color', 'w', 'Position', [100, 50, 1000, 900]);
            sgtitle('\textbf{Safety Tasks Activation Analysis}', 'Interpreter', 'latex', 'FontSize', 16);
            
            % LEFT ARM ALTITUDE
            ax1 = subplot(4, 1, 1);
            obj.plotAltitudeOverlay(ax1, t_vec, range, idx_alt_L, obj.tool_pos_L(3, range), 'LEFT');
            
            % RIGHT ARM ALTITUDE
            ax2 = subplot(4, 1, 2);
            obj.plotAltitudeOverlay(ax2, t_vec, range, idx_alt_R, obj.tool_pos_R(3, range), 'RIGHT');
            
            % LEFT ARM JOINT LIMITS
            ax3 = subplot(4, 1, 3);
            obj.plotJointLimitActivation(ax3, t_vec, range, idx_jl_L, 'LEFT');
            
            % RIGHT ARM JOINT LIMITS
            ax4 = subplot(4, 1, 4);
            obj.plotJointLimitActivation(ax4, t_vec, range, idx_jl_R, 'RIGHT');
            
            linkaxes([ax1, ax2, ax3, ax4], 'x');
        end

        function plotAltitudeOverlay(obj, ax, t, range, task_idx, alt_data, arm_name)
            if task_idx == 0, text(ax, 0.5, 0.5, ['No ALT Task: ' arm_name]); return; end
            
            raw_act = obj.a_task(task_idx, range);
            act_vec = cell2mat(raw_act); 
            
            yyaxis(ax, 'left');
            plot(ax, t, alt_data, 'b-', 'LineWidth', 1.5);
            ylabel(ax, 'Altitude Z [m]', 'FontSize', 11);
            grid(ax, 'on');
            
            yyaxis(ax, 'right');
            area(ax, t, act_vec, 'FaceColor', 'r', 'FaceAlpha', 0.3, 'EdgeColor', 'none');
            ylim(ax, [-0.1, 1.1]);
            ylabel(ax, 'Activation $\alpha$', 'Interpreter', 'latex', 'FontSize', 11);
            
            title(ax, sprintf('\\textbf{%s ARM: Minimum Altitude Task}', arm_name), 'Interpreter', 'latex');
            obj.add_phase_lines(ax, t, range);
        end

        function plotJointLimitActivation(obj, ax, t, range, task_idx, arm_name)
            if task_idx == 0, text(ax, 0.5, 0.5, ['No JL Task: ' arm_name]); return; end
            
            raw_act = obj.a_task(task_idx, range);
            act_mat = zeros(7, length(range));
            for k=1:length(range)
                val = raw_act{k};
                if length(val) == 7, act_mat(:, k) = val;
                elseif length(val) == 1, act_mat(:, k) = repmat(val, 7, 1); end
            end
            
            plot(ax, t, act_mat, 'LineWidth', 1.5);
            ylabel(ax, 'Activation $\alpha$', 'Interpreter', 'latex', 'FontSize', 11);
            ylim(ax, [-0.1, 1.1]); grid(ax, 'on');
            legend(ax, {'q1','q2','q3','q4','q5','q6','q7'}, 'NumColumns', 7, 'Location', 'SouthWest', 'FontSize', 8);
            title(ax, sprintf('\\textbf{%s ARM: Joint Limits Activation}', arm_name), 'Interpreter', 'latex');
            obj.add_phase_lines(ax, t, range);
        end

        % =========================================================
        % === 2. DETAILED VELOCITIES (Compact Plot) ===
        % =========================================================
        function plotDetailedVelocities(obj)
            if obj.data_len == 0, warning('No data to plot'); return; end
            range = 1:obj.data_len;
            t_vec = obj.t(range);
            
            % Colori per X, Y, Z
            colors = [0.8500 0.3250 0.0980;  % red (X)
                      0.4660 0.6740 0.1880;  % green (Y)
                      0 0.4470 0.7410];      % Blue (Z)
            
            % --- Helper plottare XYZ ---
            function plot_xyz_components(ax, t, data_nc, data_act, is_angular)
                hold(ax, 'on'); grid(ax, 'on');
                
                % Indici: 1-3 per Angular, 4-6 per Linear
                if is_angular, idxs = 1:3; else, idxs = 4:6; end
                
                % Plot Non-Coop (Dashed)
                if ~isempty(data_nc)
                    l1 = plot(ax, t, data_nc(idxs(1), range), '--', 'Color', colors(1,:), 'LineWidth', 1.2);
                    l2 = plot(ax, t, data_nc(idxs(2), range), '--', 'Color', colors(2,:), 'LineWidth', 1.2);
                    l3 = plot(ax, t, data_nc(idxs(3), range), '--', 'Color', colors(3,:), 'LineWidth', 1.2);
                end
                
                % Plot Actual (Solid)
                l4 = plot(ax, t, data_act(idxs(1), range), '-', 'Color', colors(1,:), 'LineWidth', 1.5);
                l5 = plot(ax, t, data_act(idxs(2), range), '-', 'Color', colors(2,:), 'LineWidth', 1.5);
                l6 = plot(ax, t, data_act(idxs(3), range), '-', 'Color', colors(3,:), 'LineWidth', 1.5);
                
                % Labels LaTeX (CORRETTE)
                if is_angular
                    ylabel(ax, 'Angular [rad/s]', 'Interpreter', 'latex', 'FontSize', 11);
                    title(ax, '\textbf{Angular Velocity} ($\omega_x, \omega_y, \omega_z$)', 'Interpreter', 'latex');
                else
                    ylabel(ax, 'Linear [m/s]', 'Interpreter', 'latex', 'FontSize', 11);
                    title(ax, '\textbf{Linear Velocity} ($\dot{x}, \dot{y}, \dot{z}$)', 'Interpreter', 'latex');
                end
                
               
                if ~isempty(data_nc)
                    legend(ax, [l4, l1], {'Actual (X,Y,Z)', 'Non-Coop (X,Y,Z)'}, 'Location', 'best');
                else
                    legend(ax, [l4, l5, l6], {'Actual X', 'Actual Y', 'Actual Z'}, 'Location', 'best');
                end
                
                obj.add_phase_lines(ax, t, range);
            end
            
            % --- FIGURE 1: LEFT ARM (Comparison) ---
            figure('Name', 'LEFT ARM: Non-Coop vs Actual', 'Color', 'w', 'Position', [50, 50, 1000, 700]);
            sgtitle('\textbf{LEFT ARM: Non-Cooperative vs Cooperative}', 'Interpreter', 'latex', 'FontSize', 16);
            
            ax1 = subplot(2, 1, 1);
            plot_xyz_components(ax1, t_vec, obj.xdot_L_nc, obj.xdot_L_act, true); % Angular
            
            ax2 = subplot(2, 1, 2);
            plot_xyz_components(ax2, t_vec, obj.xdot_L_nc, obj.xdot_L_act, false); % Linear
            
            linkaxes([ax1, ax2], 'x');

            % --- FIGURE 2: RIGHT ARM (Comparison) ---
            figure('Name', 'RIGHT ARM: Non-Coop vs Actual', 'Color', 'w', 'Position', [100, 100, 1000, 700]);
            sgtitle('\textbf{RIGHT ARM: Non-Cooperative vs Cooperative}', 'Interpreter', 'latex', 'FontSize', 16);
            
            ax3 = subplot(2, 1, 1);
            plot_xyz_components(ax3, t_vec, obj.xdot_R_nc, obj.xdot_R_act, true); % Angular
            
            ax4 = subplot(2, 1, 2);
            plot_xyz_components(ax4, t_vec, obj.xdot_R_nc, obj.xdot_R_act, false); % Linear
            
            linkaxes([ax3, ax4], 'x');

            % --- FIGURE 3: LEFT ARM (Coop Only) ---
            figure('Name', 'LEFT ARM: Cooperative Only', 'Color', 'w', 'Position', [150, 150, 1000, 700]);
            sgtitle('\textbf{LEFT ARM: Cooperative Velocity Only}', 'Interpreter', 'latex', 'FontSize', 16);
            
            ax5 = subplot(2, 1, 1);
            plot_xyz_components(ax5, t_vec, [], obj.xdot_L_act, true); % Angular
            
            ax6 = subplot(2, 1, 2);
            plot_xyz_components(ax6, t_vec, [], obj.xdot_L_act, false); % Linear
            
            linkaxes([ax5, ax6], 'x');
        end

        % =========================================================
        % === 3. Q2 ANALYSIS (Desired vs Actual + Distance) ===
        % =========================================================
        function plotAnalysisQ2(obj)
            if obj.data_len == 0, warning('Dati non sufficienti per il plot Q2.'); return; end
            range = 1:obj.data_len;
            t_vec = obj.t(range);
            
            diff_vec = obj.tool_pos_L(:, range) - obj.tool_pos_R(:, range);
            dist_norm = sqrt(sum(diff_vec.^2, 1));
            
            idx_obj_task = obj.findTaskIdx('OBJ_MOT_L');
            v_des_norm = zeros(1, length(range));
            if idx_obj_task > 0
                raw_ref = obj.xdotbar_task(idx_obj_task, range);
                for k = 1:length(range)
                    if ~isempty(raw_ref{k}), v_des_norm(k) = norm(raw_ref{k}(4:6)); end
                end
            end
            
            v_actual_norm = sqrt(sum(obj.xdot_L_act(4:6, range).^2, 1)); 
            v_actual_norm = smoothdata(v_actual_norm, 'movmean', 10);
            
            figure('Name', 'Q2 Analysis', 'Color', 'w', 'Position', [100, 100, 1000, 800]);
            
            ax1 = subplot(2,1,1); hold on; grid on;
            plot(t_vec, v_des_norm, '--', 'Color', [0.85 0.32 0.1], 'LineWidth', 2, 'DisplayName', 'Desired (Non-Coop)');
            plot(t_vec, v_actual_norm, '-', 'Color', [0 0.44 0.74], 'LineWidth', 2, 'DisplayName', 'Actual (Coop)');
            ylabel('|v| [m/s]', 'Interpreter', 'latex'); title('\textbf{Velocity Comparison}', 'Interpreter', 'latex'); legend('Location','best');
            obj.add_phase_lines(ax1, t_vec, range);
            
            ax2 = subplot(2,1,2); hold on; grid on;
            plot(t_vec, dist_norm, 'k-', 'LineWidth', 2);
            ylabel('Dist [m]', 'Interpreter', 'latex'); title('\textbf{Tool Distance}', 'Interpreter', 'latex');
            if max(dist_norm) > 0, ylim([min(dist_norm)*0.9, max(dist_norm)*1.1]); end
            obj.add_phase_lines(ax2, t_vec, range);
        end

        % --- HELPERS ---
        function idx = findTaskIdx(obj, partial_name)
            idx = 0;
            for i = 1:obj.n_tasks
                if contains(obj.tasks_ref{i}.task_name, partial_name, 'IgnoreCase', true)
                    idx = i; break;
                end
            end
        end

        function add_phase_lines(obj, h_ax, t_vec, valid_range)
            axes(h_ax); 
            yl = ylim;
            phases = obj.action_idx_log(valid_range);
            changes = find(diff(phases) ~= 0);
            phase_names = obj.action_names;
            
            prev_idx = 1;
            for i = 1:length(changes)+1
                if i <= length(changes)
                    idx = changes(i);
                    xline(t_vec(idx), '--k', 'Alpha', 0.5);
                else
                    idx = length(t_vec);
                end
                
                center_t = (t_vec(prev_idx) + t_vec(idx)) / 2;
                if prev_idx <= length(phases)
                    p_val = phases(prev_idx);
                    if p_val <= length(phase_names), txt = phase_names{p_val};
                    else, txt = sprintf('P%d', p_val); end
                    
                    text(center_t, yl(2), txt, 'HorizontalAlignment', 'center', ...
                        'VerticalAlignment', 'top', 'Interpreter', 'latex', 'FontSize', 8, ...
                        'BackgroundColor', 'w', 'EdgeColor', 'none', 'Margin', 1);
                end
                prev_idx = idx + 1;
            end
        end
        
        function plotToolDistance(obj), obj.plotAnalysisQ2(); end
        function plotAll(obj, ~), disp('Usa i metodi specifici (plotSafetyTasks, plotDetailedVelocities).'); end
    end
end