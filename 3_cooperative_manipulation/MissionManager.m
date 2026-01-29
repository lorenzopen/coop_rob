classdef MissionManager < handle
    properties
        % Soglie di tolleranza
        pos_threshold = 0.01;   % 1 cm di errore
        ang_threshold = 0.1;    % ~5.7 gradi
        
        missionPhase = 1;       % Fase corrente della missione
    end

    methods
        function obj = MissionManager()
        end

        function updateMissionPhase(obj, mgrL, mgrR, coop_system)
            % INPUT: 
            % - mgrL: ActionManager braccio sx
            % - mgrR: ActionManager braccio dx
            % - coop_system: Sistema simulato

            switch obj.missionPhase
                
                case 1 % --- PHASE 1: REACH GRASPING POINTS ---
                    
                    % Recuperiamo i task cartesiani di avvicinamento
                    taskL = mgrL.getTaskByName('CART_L');
                    taskR = mgrR.getTaskByName('CART_R');
                    
                    is_L_ready = false;
                    is_R_ready = false;

                    % Check convergenza sx
                    if ~isempty(taskL) && ~isempty(taskL.xdotbar)
                        v_ang = norm(taskL.xdotbar(1:3));
                        v_lin = norm(taskL.xdotbar(4:6));
                        if v_lin < obj.pos_threshold && v_ang < obj.ang_threshold
                            is_L_ready = true;
                        end
                    end
                    
                    % Check convergenza dx
                    if ~isempty(taskR) && ~isempty(taskR.xdotbar)
                        v_ang = norm(taskR.xdotbar(1:3));
                        v_lin = norm(taskR.xdotbar(4:6));
                        if v_lin < obj.pos_threshold && v_ang < obj.ang_threshold
                            is_R_ready = true;
                        end
                    end

                    % TRANSIZIONE ->  2 (cooperation)
                    if is_L_ready && is_R_ready
                        fprintf('\n[MissionManager] Both arms reached grasping points. Starting Cooperation.\n');
                        
                        % 1. (attach virtual object)
                        coop_system.left_arm.compute_object_frame();
                        coop_system.right_arm.compute_object_frame();
                        
                        % 2. 
                        % (Nel main: "Coop Calc Left/Right")
                        mgrL.setBinaryTransition(true); 
                        mgrR.setBinaryTransition(true);
                        
                        mgrL.setCurrentAction("Coop Calc Left");
                        mgrR.setCurrentAction("Coop Calc Right");
                        
                        obj.missionPhase = 2;
                    end

                case 2 % --- PHASE 2: COOPERATIVE MANIPULATION ---
                    
                    
                    % Calcolo errore cartesiano corrente dell'oggetto (Left arm come riferimento master)
                    [err_ang, err_lin] = CartError(coop_system.left_arm.wTog, coop_system.left_arm.wTo);
                    
                    dist_lin = norm(err_lin);
                    dist_ang = norm(err_ang);
                    
                    % TRANSIZIONE -> FASE 3 (STOP)
                    if dist_lin < obj.pos_threshold && dist_ang < obj.ang_threshold
                        fprintf('\n[MissionManager] Object Goal Reached. Stopping Motion.\n');
                        
                        % Switch Action Managers all'azione di Stop
                        mgrL.setBinaryTransition(false); % Soft stop
                        mgrR.setBinaryTransition(false);
                        
                        mgrL.setCurrentAction("Stop Left");
                        mgrR.setCurrentAction("Stop Right");
                        
                        obj.missionPhase = 3;
                    end

                case 3 % --- PHASE 3: STOP ---
                    % Missione completata, manteniamo lo stato di stop
            end
        end
    end
end