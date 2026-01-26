% Add paths
addpath('./simulation_scripts');
addpath('./tools');
addpath('./icat');
addpath('./robust_robot');
addpath('./tasks/')
clc; clear; close all;

% Simulation parameters
dt       = 0.005;
endTime  = 40;
% Initialize robot model and simulator
robotModel = UvmsModel();
sim = UvmsSim(dt, robotModel, endTime);
% Initialize Unity interface
unity = UnityInterface("127.0.0.1");

% Define tasks
task_tool = TaskTool();
task_horizontal = TaskHorizontalAttitude();
task_altitude = TaskAltitude();
task_position = TaskPosition();
task_orientation = TaskOrientation();
task_heading = TaskAlign();
task_land = TaskLand();
task_stop = TaskStopVehicle();


task_navigation_set = { task_altitude, task_orientation, task_position };        % Navigation
task_landing_set = { task_horizontal, task_heading, task_land, task_position };  % Landing
task_manipulation_set = { task_stop, task_tool };                                % Manipulation

% Unifying task list
unified_task_list = {task_altitude, task_stop, task_horizontal, task_heading, task_land, task_orientation, task_position, task_tool };

IDX_NAV   = 1;
IDX_LAND  = 2;
IDX_MANIP = 3;

% Define actions and add to ActionManager
actionManager = ActionManager();
actionManager.addAction(task_navigation_set);  
actionManager.addAction(task_landing_set);          
actionManager.addAction(task_manipulation_set);    
actionManager.addUnifiedList(unified_task_list);

%disp(actionManager.actionsName)



% Define desired positions and orientations (world frame)
w_arm_goal_position = [12.2025, 37.3748, -39.8860]'; % nodule position
w_arm_goal_orientation = [0, pi, pi/2];
w_vehicle_goal_position = [10.5 37.5 -38]';
w_vehicle_goal_orientation = [0, -0.06, 0.5];

% Set current action
actionManager.setCurrentAction(IDX_NAV);
robotModel.setGoal(w_arm_goal_position, w_arm_goal_orientation, w_vehicle_goal_position, w_vehicle_goal_orientation);

% Initialize the logger
logger = SimulationLogger(ceil(endTime/dt)+1, robotModel, task_navigation_set);
idx=1;

% Initialize mission phase: 1=Navigation, 2=Landing, 3=Manipulation
missionPhase = 1;
manFlag = false; % manipulation complete flag for logging
goalReset = false;


%---------------------------------------------------------------------
%---------------------------------------------------------------------
% da cambiare come fare workspace check del braccio, cosi troppo specifico
%---------------------------------------------------------------------
%---------------------------------------------------------------------


% compute arm max reach
%rmax = computeArmMaxReach(robotModel);
%fprintf('arm max reach (m): %f\n', rmax);
%rmax = 1.855736; % precomputed value using computeArmMaxReach function
rmax = 1.3; % set smaller value to force vehicle repositioning

% Main simulation loop
for step = 1:sim.maxSteps
    % 1. Receive altitude from Unity
    robotModel.altitude = unity.receiveAltitude(robotModel);
    % --- MISSION CONTROLLER ---
    if missionPhase == 1
        % --- PHASE 1: SAFE NAVIGATION ---
        % Obiettivo: Avvicinarsi alla zona (XY) e allinearsi
        %xy_error = norm(robotModel.eta(1:2) - w_vehicle_goal_position(1:2));
        % Calcolo errore orientamento (Yaw: eta(6) vs goal(6))
        % Nota: Normale differenza angolare
        %yaw_err = abs(angdiff(robotModel.eta(6), w_vehicle_goal_orientation(3)));
        %if xy_error < 0.2 && yaw_err < 0.1
        pitch_error = task_orientation.error_vec(2); 
        yaw_error = task_orientation.error_vec(3);
        if ((task_position.error < 0.2) & (pitch_error < 0.01) & (yaw_error < 0.1))
            disp("Navigation complete - switch to Landing");
            actionManager.setCurrentAction(IDX_LAND);
            missionPhase = 2;
        end
    elseif missionPhase == 2
        % --- PHASE 2: LANDING & REACHABILITY CHECK ---
        % Obiettivo: Scendere a quota operativa e verificare se il nodulo è raggiungibile
        alt_error = abs(robotModel.altitude - 0.5); % Target altitude es. 0.5m
        % Calcoli per raggiungibilità (Reachability)
        w_veh_pos = robotModel.eta(1:2);
        w_arm_goal_pos_2d = w_arm_goal_position(1:2);
        % Vettore distanza Veicolo -> Oggetto
        d_vec = w_arm_goal_pos_2d - w_veh_pos;
        d = norm(d_vec);
        % Errore XY residuo rispetto al target del veicolo attuale
        xy_error = norm(robotModel.eta(1:2) - w_vehicle_goal_position(1:2));
 
        % Se siamo atterrati e allineati
        if alt_error < 0.1
            % Caso A: Nodulo troppo lontano (fuori workspace)
            if d > rmax && ~goalReset
                goalReset = true;
                fprintf('Adjusting vehicle goal to guarantee nodule reachability (dist=%.2f)\n', d);
                % Calcola correzione: ci avviciniamo lungo la linea d_vec
                % in modo che la distanza finale sia rmax
                correction = d_vec * (1 - rmax/d); 
                % Aggiorniamo il goal del veicolo
                w_vehicle_goal_position(1:2) = w_vehicle_goal_position(1:2) + correction;
                % Inviamo il nuovo goal al robot
                robotModel.setGoal(w_arm_goal_position, w_arm_goal_orientation, ...
                                   w_vehicle_goal_position, w_vehicle_goal_orientation);
            % Caso B: Nodulo raggiungibile e veicolo posizionato
            elseif xy_error < 0.2
                 disp("Landing complete & Reachable - switch to Manipulation");
                 % Assumiamo tu abbia definito un'azione chiamata "Manipulation"
                 % Se nel codice originale era "Task Move To", cambia il nome qui sotto
                 actionManager.setCurrentAction(IDX_MANIP); 
                 missionPhase = 3;
            end
        end
    elseif missionPhase == 3
        % --- PHASE 3: MANIPULATION ---
        % Obiettivo: Portare il tool sul goal
        tool_pos_error = norm(robotModel.wTt(1:3,4) - robotModel.wTg(1:3,4));
        if tool_pos_error < 0.02 && ~manFlag
            manFlag = true;
            disp("Manipulation complete!");
            % break; % Scommenta se vuoi terminare la simulazione qui
        end
    end

    % 2. Compute control commands for current action
    [v_nu, q_dot] = actionManager.computeICAT(robotModel, dt);

    % 3. Step the simulator (integrate velocities)
    sim.step(v_nu, q_dot);

    % 4. Send updated state to Unity
    unity.send(robotModel);

    % 5. Logging
    logger.update(sim.time, sim.loopCounter);

    % 6. Optional debug prints
    if mod(sim.loopCounter, round(0.3 / sim.dt)) == 0
        fprintf('--- t = %.2f s ---\n', sim.time);
        fprintf('Alt: %.2f m\n', robotModel.altitude);
        
        % Estrai orientamento attuale (RPY) dallo stato eta
        current_roll  = robotModel.eta(4);
        current_pitch = robotModel.eta(5);
        current_yaw   = robotModel.eta(6);
        
        % Stampa confronto Attuale vs Goal
        fprintf('ORIENTAMENTO [Roll, Pitch, Yaw] (rad):\n');
        fprintf('  Attuale: [%.4f,  %.4f,  %.4f]\n', current_roll, current_pitch, current_yaw);
        fprintf('  Goal:    [%.4f,  %.4f,  %.4f]\n', w_vehicle_goal_orientation(1), w_vehicle_goal_orientation(2), w_vehicle_goal_orientation(3));
        
        % Calcola e stampa errore semplice
        err_r = w_vehicle_goal_orientation(1) - current_roll;
        err_p = w_vehicle_goal_orientation(2) - current_pitch;
        err_y = w_vehicle_goal_orientation(3) - current_yaw;
        fprintf('  Errore:  [%.4f,  %.4f,  %.4f]\n', err_r, err_p, err_y);
        
        % if missionPhase == 1
        %     pos_error = norm(robotModel.eta(1:2) - w_vehicle_goal_position(1:2));
        %     fprintf('Vehicle position error (m): %.3f\n\n', pos_error);
        % elseif missionPhase == 2
        %     fprintf('Heading error (rad): %.3f\n', robotModel.err_angle);
        %     fprintf('Vehicle position error (m): %.3f\n\n', pos_error);
        % elseif missionPhase == 3
        %     tool_pos_error = norm(robotModel.wTt(1:3,4) - robotModel.wTg(1:3,4));
        %     fprintf('Tool position error (m): %.3f\n \n', tool_pos_error);
        % end
    end
    % 7. Optional real-time slowdown
    SlowdownToRealtime(dt);
end

% Display plots
logger.plotAll();

% Clean up Unity interface
delete(unity);