classdef SimulationLogger < handle
    properties
        t               % time vector
        ql, qr          % posizioni
        qdotl, qdotr    % velocità reali (Coop)
        xdot_L_nc, xdot_L_act      
        xdot_R_nc, xdot_R_act      
        xdotbar_task    
        a_task          
        tasks_ref       
        n_tasks         
        action_idx_log  
        action_names    
        tool_pos_L      
        tool_pos_R      
        robot           
        data_len        
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
            obj.xdot_L_nc = zeros(6, maxLoops); obj.xdot_L_act = zeros(6, maxLoops);
            obj.xdot_R_nc = zeros(6, maxLoops); obj.xdot_R_act = zeros(6, maxLoops);
            obj.tool_pos_L = zeros(3, maxLoops); obj.tool_pos_R = zeros(3, maxLoops);
            obj.xdotbar_task = cell(obj.n_tasks, maxLoops);
            obj.a_task = cell(obj.n_tasks, maxLoops);
            obj.action_idx_log = ones(1, maxLoops); 
        end
        
        function update(obj, t, loop, mission_phase, ql_nc, qr_nc)
            if loop <= 0, return; end
            obj.data_len = loop;
            obj.t(loop) = t;
            
            % Dati Giunti
            obj.ql(:, loop) = obj.robot.left_arm.q;
            obj.qr(:, loop) = obj.robot.right_arm.q;
            obj.qdotl(:, loop) = obj.robot.left_arm.qdot;
            obj.qdotr(:, loop) = obj.robot.right_arm.qdot;
            
            % Velocità Cartesiane
            J_L = obj.robot.left_arm.wJo; if isempty(J_L), J_L = obj.robot.left_arm.wJt; end
            J_R = obj.robot.right_arm.wJo; if isempty(J_R), J_R = obj.robot.right_arm.wJt; end
            
            if ~isempty(ql_nc), obj.xdot_L_nc(:, loop) = J_L * ql_nc; end
            if ~isempty(qr_nc), obj.xdot_R_nc(:, loop) = J_R * qr_nc; end
            
            obj.xdot_L_act(:, loop) = J_L * obj.robot.left_arm.qdot;
            obj.xdot_R_act(:, loop) = J_R * obj.robot.right_arm.qdot;
            
            % Posizioni Tool
            obj.tool_pos_L(:, loop) = obj.robot.left_arm.wTt(1:3, 4);
            obj.tool_pos_R(:, loop) = obj.robot.right_arm.wTt(1:3, 4);
        
            % Task Data
            for i = 1:obj.n_tasks
                ct = obj.tasks_ref{i};
                obj.xdotbar_task{i, loop} = ct.xdotbar;
                if isprop(ct, 'A')
                    if ismatrix(ct.A) && size(ct.A,1) == size(ct.A,2) && size(ct.A,1) > 1
                        obj.a_task{i, loop} = diag(ct.A); % Salva la diagonale se matrice
                    else
                        obj.a_task{i, loop} = ct.A; 
                    end
                else
                    obj.a_task{i, loop} = 1; 
                end
            end
            if isempty(mission_phase), mission_phase = 1; end
            obj.action_idx_log(loop) = mission_phase;
        end

        % =========================================================
        % === 1. SAFETY TASKS (ALTITUDE + JOINT LIMITS) ===
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
            
            % 1. LEFT ARM ALTITUDE
            ax1 = subplot(4, 1, 1);
            obj.plotAltitudeOverlay(ax1, t_vec, range, idx_alt_L, obj.tool_pos_L(3, range), 'LEFT');
            
            % 2. RIGHT ARM ALTITUDE
            ax2 = subplot(4, 1, 2);
            obj.plotAltitudeOverlay(ax2, t_vec, range, idx_alt_R, obj.tool_pos_R(3, range), 'RIGHT');
            
            % 3. LEFT ARM JOINT LIMITS (Detailed)
            ax3 = subplot(4, 1, 3);
            obj.plotJointLimitActivation(ax3, t_vec, range, idx_jl_L, 'LEFT');
            
            % 4. RIGHT ARM JOINT LIMITS (Detailed)
            ax4 = subplot(4, 1, 4);
            obj.plotJointLimitActivation(ax4, t_vec, range, idx_jl_R, 'RIGHT');
            
            linkaxes([ax1, ax2, ax3, ax4], 'x');
        end

        % Helper per Altitude (con fix per Area error)
        function plotAltitudeOverlay(obj, ax, t, range, task_idx, alt_data, arm_name)
            % Asse SX: Altitudine
            yyaxis(ax, 'left');
            plot(ax, t, alt_data, 'b-', 'LineWidth', 1.5);
            ylabel(ax, 'Alt Z [m]'); grid(ax, 'on'); hold(ax, 'on');
            
            % Asse DX: Attivazione
            yyaxis(ax, 'right');
            if task_idx > 0
                % Estrazione sicura scalare
                act_vec = zeros(1, length(range));
                for k = 1:length(range)
                    val = obj.a_task{task_idx, range(k)};
                    if ~isempty(val), act_vec(k) = val(1); end
                end
                % FIX: t(:) e act_vec(:) forzano vettori colonna
                area(ax, t(:), act_vec(:), 'FaceColor', 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
            end
            ylabel(ax, 'Act $\alpha$', 'Interpreter', 'latex'); ylim(ax, [-0.1, 1.1]);
            title(ax, sprintf('\\textbf{%s ARM: Altitude Task}', arm_name), 'Interpreter', 'latex');
            obj.add_phase_lines(ax, t, range);
        end

        % Helper per Joint Limits (Focus richiesto)
        function plotJointLimitActivation(obj, ax, t, range, task_idx, arm_name)
            if task_idx == 0
                text(ax, 0.5, 0.5, ['Task Not Found: ' arm_name], 'HorizontalAlignment', 'center'); 
                return; 
            end
            
            % Estrazione sicura attivazioni (7 DOF)
            act_mat = zeros(7, length(range));
            for k = 1:length(range)
                val = obj.a_task{task_idx, range(k)};
                if length(val) == 7
                    act_mat(:, k) = val(:);
                elseif length(val) == 1
                    act_mat(:, k) = repmat(val, 7, 1);
                end
            end
            
            % Plot (Trasposta act_mat per avere il tempo sulle righe nel plot)
            plot(ax, t, act_mat', 'LineWidth', 1.5);
            
            ylabel(ax, 'Act $\alpha$', 'Interpreter', 'latex');
            ylim(ax, [-0.1, 1.1]); grid(ax, 'on');
            
            % Legenda per i 7 giunti
            legend(ax, {'q1','q2','q3','q4','q5','q6','q7'}, 'NumColumns', 7, 'Location', 'SouthWest', 'FontSize', 8);
            title(ax, sprintf('\\textbf{%s ARM: Joint Limits Activation}', arm_name), 'Interpreter', 'latex');
            obj.add_phase_lines(ax, t, range);
        end

        % =========================================================
        % === 2. DETAILED VELOCITIES ===
        % =========================================================
        function plotDetailedVelocities(obj)
            if obj.data_len == 0, return; end
            range = 1:obj.data_len; t_vec = obj.t(range);
            
            % Plot semplificato Left/Right Actual Velocities
            figure('Name', 'Detailed Velocities', 'Color', 'w');
            subplot(2,1,1);
            plot(t_vec, obj.xdot_L_act(:, range)'); title('Left Arm End-Effector Velocity'); grid on;
            obj.add_phase_lines(gca, t_vec, range);
            
            subplot(2,1,2);
            plot(t_vec, obj.xdot_R_act(:, range)'); title('Right Arm End-Effector Velocity'); grid on;
            obj.add_phase_lines(gca, t_vec, range);
        end
        
        % =========================================================
        % === 3. HELPERS ===
        % =========================================================
        function plotToolDistance(obj)
            range = 1:obj.data_len;
            dist = sqrt(sum((obj.tool_pos_L(:,range) - obj.tool_pos_R(:,range)).^2, 1));
            figure('Name', 'Tool Distance'); plot(obj.t(range), dist); title('Distance'); grid on;
            obj.add_phase_lines(gca, obj.t(range), range);
        end

        function idx = findTaskIdx(obj, partial_name)
            idx = 0;
            for i = 1:obj.n_tasks
                if contains(obj.tasks_ref{i}.task_name, partial_name, 'IgnoreCase', true)
                    idx = i; break;
                end
            end
        end

        function add_phase_lines(obj, h_ax, t_vec, valid_range)
            axes(h_ax); hold on;
            phases = obj.action_idx_log(valid_range);
            changes = find(diff(phases) ~= 0);
            for i = 1:length(changes)
                xline(t_vec(changes(i)), '--k', 'Alpha', 0.5);
            end
        end

        % === PLOT ALL (Senza Coop Constraint) ===
        function plotAll(obj)
            obj.plotSafetyTasks();          % Mostra JL e Altitudine
            obj.plotDetailedVelocities();   % Velocità cartesiane
            obj.plotToolDistance();         % Distanza tra i tool
        end
        
        function plotAnalysisQ2(obj), obj.plotToolDistance(); end
    end
end