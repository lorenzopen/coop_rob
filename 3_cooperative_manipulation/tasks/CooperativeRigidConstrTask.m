classdef CooperativeRigidConstrTask < Task
    % CooperativeRigidConstrTask (Esercizio 3)
    % In un sistema decentralizzato, questo task impone al singolo robot
    % di muovere l'oggetto alla velocità cooperativa concordata (v_coop).
    %
    % Matematicamente: J_obj_local * qdot = v_coop (+ correzione drift)
    
    properties
        kp = 1.0; % Guadagno proporzionale per correggere il drift cinematico
    end

    methods
        function obj = CooperativeRigidConstrTask(robotID, taskID)
            obj.ID = robotID;
            obj.task_name = taskID;
        end
        
        function updateReference(obj, robot)
            % INPUT: 'robot' è la struttura del SINGOLO braccio
            
            % 1. Recupero la velocità cooperativa
            % Modifica: Usa isprop invece di isfield per le classi
            if isprop(robot, 'v_coop')
                v_des = robot.v_coop;
            else
                % Fallback se la proprietà non esiste (non dovrebbe accadere se panda_arm è aggiornato)
                v_des = zeros(6,1);
                warning('v_coop non trovata in panda_arm. Assicurati di aver aggiunto la proprietà in panda_arm.m');
            end

            % 2. Correzione del Drift (Opzionale ma raccomandata)
            if isprop(robot, 'wTo_partner')
                [err_ang, err_lin] = CartError(robot.wTo_partner, robot.wTo);
                v_drift = obj.kp * [err_ang; err_lin];
            else
                v_drift = zeros(6,1);
            end

            % 3. Velocità di riferimento totale
            obj.xdotbar = v_des + v_drift;
        end
        
        function updateJacobian(obj, robot)        
            % Questo Jacobiano è identico a quello dell'ObjectTask:
            % mappa le velocità di giunto del singolo robot (7) 
            % nelle velocità del frame dell'oggetto (6).
            
            % 1. Jacobiano alla flangia
            J_tool = robot.wJt;

            % 2. Trasporto sull'oggetto
            % Vettore r = Tool -> Oggetto
            r_to = robot.wTo(1:3, 4) - robot.wTt(1:3, 4);
            S_r = skew(r_to);
            
            % Matrice di trasporto (Assuming [omega; vel])
            % Se la convenzione è [vel; omega], scambiare le righe
            X_to = [eye(3), zeros(3); 
                   -S_r,    eye(3)];
            
            % 3. Jacobiano 6x7 (Decentralizzato)
            obj.J = X_to * J_tool;
        end
        
        function updateActivation(obj, robot)
            % Il vincolo rigido è sempre attivo durante la fase di trasporto
            obj.A = eye(6);
        end
    end
end