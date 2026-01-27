classdef BimanualManipulatorManager < handle
    properties
        % Soglie di tolleranza per i task
        pos_threshold = 0.01;   % 1 cm
        ang_threshold = 0.1;    % ~5.7 gradi
        
        % Stato della macchina a stati
        % 1: GoToPosition (Avvicinamento)
        % 2: Bimanual Manipulation (Trasporto oggetto)
        % 3: Stop Motion (Frenata/Fine)
        missionPhase = 1; 
    end

    methods
        function obj = BimanualManipulatorManager()
            % Costruttore
        end

        function updateMissionPhase(obj, actionManager, bm_sim)
            
            switch obj.missionPhase
                
                case 1 % --- STATO 1: GoToPosition ---
                    % Calcolo errori lineari per entrambi i bracci
                    err_pos_L = norm(bm_sim.left_arm.wTt(1:3,4) - bm_sim.left_arm.wTg(1:3,4));
                    err_pos_R = norm(bm_sim.right_arm.wTt(1:3,4) - bm_sim.right_arm.wTg(1:3,4));
                    
                    % Calcolo errori angolari
                    R_err_L = bm_sim.left_arm.wTg(1:3,1:3)' * bm_sim.left_arm.wTt(1:3,1:3);
                    R_err_R = bm_sim.right_arm.wTg(1:3,1:3)' * bm_sim.right_arm.wTt(1:3,1:3);
                    
                    % Utilizzo di un clamp per evitare errori numerici (NaN) con acos
                    err_ang_L = acos(max(min((trace(R_err_L) - 1) / 2, 1), -1));
                    err_ang_R = acos(max(min((trace(R_err_R) - 1) / 2, 1), -1));

                    % Verifica se entrambi i bracci sono in posizione
                    if max(err_pos_L, err_pos_R) < obj.pos_threshold && ...
                       max(err_ang_L, err_ang_R) < obj.ang_threshold
                        
                        fprintf('\n[BimanualManager] Grasping points reached. Switching to Manipulation.\n');
                        
                        % Calcolo dei frame dell'oggetto basati sulla posizione attuale dei tool
                        bm_sim.left_arm.compute_object_frame();
                        bm_sim.right_arm.compute_object_frame();
                        
                        % Configurazione ActionManager per la fase cooperativa
                        % Nota: Usiamo BinaryTransition perché il vincolo rigido è "on/off"
                        actionManager.setBinaryTransition(true);
                        actionManager.setCurrentAction("Bimanual Manipulation");
                        
                        % Transizione di stato
                        obj.missionPhase = 2;
                    end

                case 2 % --- STATO 2: Bimanual Manipulation ---
                    % Verifica l'errore tra la posizione attuale dell'oggetto e il suo goal
                    % Usiamo il frame dell'oggetto calcolato dal braccio sinistro (sono solidali)
                    obj_err = norm(bm_sim.left_arm.wTo(1:3,4) - bm_sim.left_arm.wTog(1:3,4));
                    
                    if obj_err < obj.pos_threshold
                        fprintf('\n[BimanualManager] Object goal reached. Switching to Stop Motion.\n');
                        
                        % Torniamo a transizioni Smooth per fermare il robot dolcemente
                        actionManager.setBinaryTransition(false);
                        actionManager.setCurrentAction("Stop Motion");
                        
                        % Transizione di stato
                        obj.missionPhase = 3;
                    end

                case 3 % --- STATO 3: Stop Motion ---
                    % Il robot esegue i task di Stop (velocità zero)
                    % È possibile aggiungere una condizione di spegnimento qui se necessario
            end
        end
    end
end