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
task_hor_attitude = TaskHorizontalAttitude();
task_vehicle_alt = TaskAltitude();
task_vehicle_pos = TaskPosition();
task_vehicle_ori = TaskOrientation();
task_vehicle_heading = TaskAlign();
task_vehicle_land = TaskLand();
task_stop_vehicle = TaskStopVehicle();


task_set1 = { task_vehicle_alt, task_vehicle_ori, task_vehicle_pos };   % Safe Navigation
task_set2 = { task_hor_attitude, task_vehicle_heading, task_vehicle_land, task_vehicle_pos };  % Landing
task_set3 = { task_stop_vehicle, task_tool };  % Manipulation

% Unifying task list
unified_task_list = {task_vehicle_alt, task_stop_vehicle, ... % safety
    task_hor_attitude, task_vehicle_heading, ... % prerequisites
    task_vehicle_land, task_vehicle_ori, task_vehicle_pos, task_tool }; % action defining

% Define actions and add to ActionManager
actionManager = ActionManager();
actionManager.addAction(task_set1, "Safe Navigation");  % Action 1: Safe Navigation
actionManager.addAction(task_set2, "Landing");          % Action 2: Landing
actionManager.addAction(task_set3, "Manipulation");     % Action 3: Manipulation
actionManager.addUnifyingTaskList(unified_task_list);

disp(actionManager.actionsName)

% Set current action
actionManager.setCurrentAction("Safe Navigation");

% Define desired positions and orientations (world frame)
w_arm_goal_position = [12.2025, 37.3748, -39.8860]'; % nodule position
w_arm_goal_orientation = [0, pi, pi/2];

% Vehicle goal position and orientation
w_vehicle_goal_position = [10.5 37.5 -38]';
w_vehicle_goal_orientation = [0, -0.06, 0.5];

% Set goals in the robot model
robotModel.setGoal(w_arm_goal_position, w_arm_goal_orientation, w_vehicle_goal_position, w_vehicle_goal_orientation);

% Initialize the logger
logger = SimulationLogger(ceil(endTime/dt)+1, robotModel, task_set1);
idx=1;

% Initialize mission phase: 1=Safe Navigation, 2=Landing, 3=Manipulation
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

    % Mission controller
    if missionPhase == 1
        xy_error = norm(robotModel.eta(1:2) - w_vehicle_goal_position(1:2));
        ori_error = task_vehicle_ori.error;

        if xy_error < 0.1 && ori_error < 0.1
        disp("Safe Navigation complete (Pos & Ori OK) - switch to Landing")
        actionManager.setCurrentAction("Landing");
        missionPhase = 2;
        end
    elseif missionPhase == 2
        alt_error = abs(robotModel.altitude - 0.5);
        % to check if nodule is reachable
        w_vehicle_position = robotModel.eta(1:2);
        d_vec = w_arm_goal_position(1:2) - w_vehicle_position;
        d = norm(d_vec);

        xy_error = norm(robotModel.eta(1:2) - w_vehicle_goal_position(1:2));

        if alt_error < 0.1 && abs(robotModel.err_angle) < 0.1
            % Vehicle is landed and aligned
            if d > rmax && ~goalReset
                goalReset = true;  
                % Nodule still outside workspace -> adjust vehicle goal
                fprintf('Adjusting vehicle goal to guarantee nodule reachability\n');
                correction = d_vec * (d - rmax)/d;
                w_vehicle_goal_position(1:2) = w_vehicle_goal_position(1:2) + correction;
                robotModel.setGoal(w_arm_goal_position, w_arm_goal_orientation, w_vehicle_goal_position, w_vehicle_goal_orientation);
                % stay in missionPhase 2 until vehicle reaches new goal
            elseif xy_error < 0.2
                % Nodule is within reach -> switch to Manipulation
                disp("Landing complete - switch to Manipulation")
                actionManager.setCurrentAction("Manipulation");
                missionPhase = 3;
            end
        end
    elseif missionPhase == 3
        tool_pos_error = norm(robotModel.wTt(1:3,4) - robotModel.wTg(1:3,4));
        if tool_pos_error < 0.01 && ~manFlag
            manFlag = true;
            disp("Manipulation complete");
            % Optionally, end simulation or hold position
            % break;
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
        
        if missionPhase == 1
            pos_error = norm(robotModel.eta(1:2) - w_vehicle_goal_position(1:2));
            fprintf('Vehicle position error (m): %.3f\n\n', pos_error);
        elseif missionPhase == 2
            fprintf('Heading error (rad): %.3f\n', robotModel.err_angle);
            fprintf('Vehicle position error (m): %.3f\n\n', pos_error);
        elseif missionPhase == 3
            tool_pos_error = norm(robotModel.wTt(1:3,4) - robotModel.wTg(1:3,4));
            fprintf('Tool position error (m): %.3f\n \n', tool_pos_error);
        end
    end
    % 7. Optional real-time slowdown
    SlowdownToRealtime(dt);
end

% Display plots
logger.plotAll();

% Clean up Unity interface
delete(unity);