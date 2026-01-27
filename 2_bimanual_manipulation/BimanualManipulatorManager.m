classdef BimanualManipulatorManager < handle
    properties
        % Soglie di tolleranza per i task
        pos_threshold = 0.01;   % 1 cm (o 1 cm/s)
        ang_threshold = 0.1;    % ~5.7 gradi (o 0.1 rad/s)
        
        missionPhase = 1; 
    end

    methods
        function obj = BimanualManipulatorManager()
            % Costruttore
        end

        function updateMissionPhase(obj, actionManager, bm_sim)
            
            switch obj.missionPhase
                
                case 1 % --- STATO 1: GoToPosition (Reaching) ---
                    
                    taskL = actionManager.getTaskByName('LT');
                    taskR = actionManager.getTaskByName('RT');
                    
                    is_L_done = false;
                    is_R_done = false;
                    
                    % 2. Controllo convergenza Braccio Sinistro
                    if ~isempty(taskL) && ~isempty(taskL.xdotbar)
                        vel_ang_L = norm(taskL.xdotbar(1:3));
                        vel_lin_L = norm(taskL.xdotbar(4:6));
                        
                        if vel_lin_L < obj.pos_threshold && vel_ang_L < obj.ang_threshold
                            is_L_done = true;
                        end
                    end
                    
                    % 3. Controllo convergenza Braccio Destro
                    if ~isempty(taskR) && ~isempty(taskR.xdotbar)
                        vel_ang_R = norm(taskR.xdotbar(1:3));
                        vel_lin_R = norm(taskR.xdotbar(4:6));
                        
                        if vel_lin_R < obj.pos_threshold && vel_ang_R < obj.ang_threshold
                            is_R_done = true;
                        end
                    end

                    % 4. Transizione
                    if is_L_done && is_R_done
                        fprintf('\n[BimanualManager] Grasping points reached. Switching to Manipulation.\n');
                        
                        % Calcoli per la presa
                        bm_sim.left_arm.compute_object_frame();
                        bm_sim.right_arm.compute_object_frame();
                        
                        % Cambio azione
                        actionManager.setBinaryTransition(true);
                        actionManager.setCurrentAction("Bimanual Manipulation");
                        
                        obj.missionPhase = 2;
                    end

                case 2 % --- STATO 2: Bimanual Manipulation ---
                    
                    % Recuperiamo il task oggetto usando il nome definito nel main ("OBJECT_MOTION_L")
                    % Nota: Basta controllarne uno poiché sono coordinati dal vincolo rigido
                    taskObj = actionManager.getTaskByName('OBJECT_MOTION_L');
                    
                    if ~isempty(taskObj) && ~isempty(taskObj.xdotbar)
                        % Controlliamo la velocità lineare
                        vel_lin_obj = norm(taskObj.xdotbar(4:6));
                        
                        if vel_lin_obj < obj.pos_threshold
                            fprintf('\n[BimanualManager] Object goal reached. Switching to Stop Motion.\n');
                            
                            actionManager.setBinaryTransition(false);
                            actionManager.setCurrentAction("Stop Manipulation");
                            
                            obj.missionPhase = 3;
                        end
                    end

                case 3 % --- STATO 3: Stop Motion ---
                    % Fine
            end
        end
    end
end