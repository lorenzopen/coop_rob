% Main simulation loop
for step = 1:sim.maxSteps
    % 1. Receive altitude from Unity
    robotModel.altitude = unity.receiveAltitude(robotModel);
    % --- MISSION CONTROLLER ---
    if missionPhase == 1
        % --- PHASE 1: SAFE NAVIGATION ---
        % Obiettivo: Avvicinarsi alla zona (XY) e allinearsi
        xy_error = norm(robotModel.eta(1:2) - w_vehicle_goal_position(1:2));
        % Calcolo errore orientamento (Yaw: eta(6) vs goal(6))
        % Nota: Normale differenza angolare
        yaw_err = abs(angdiff(robotModel.eta(6), w_vehicle_goal_orientation(3)));
        if xy_error < 0.2 && yaw_err < 0.1
            disp("Safe Navigation complete (Pos & Ori OK) - switch to Landing");
            actionManager.setCurrentAction("Landing");
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
                 actionManager.setCurrentAction("Manipulation"); 
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