classdef JointLimitTask < Task
    % JointLimitTask per Esercizio 3 (Cooperative - Decentralized)
    % Questo task agisce su un singolo manipolatore (7 DOF).
    
    properties
        delta = 0.2; % Margine di sicurezza in radianti
        gain = 0.5;  % Guadagno per il calcolo della velocità di riferimento
    end

    methods
        function obj = JointLimitTask(robot_ID, taskID, delta)
            obj.ID = robot_ID;
            obj.task_name = taskID;
            if nargin > 2
                obj.delta = delta;
            end
        end

        function updateReference(obj, robot)
            % INPUT: 'robot' è la struttura del SINGOLO braccio (non l'intero sistema)
            
            % Calcolo dell'errore rispetto ai limiti (vettoriale)
            % jlmin e jlmax sono presi dal datasheet [cite: 102]
            err_min = (robot.jlmin + obj.delta) - robot.q;
            err_max = robot.q - (robot.jlmax - obj.delta);

            % Calcoliamo la velocità di riferimento
            % Se err < 0, il giunto è in zona sicura -> qdot = 0
            % Se err > 0, il giunto deve essere spinto indietro
            qdot_ref = obj.gain * (max(err_min, 0) - max(err_max, 0));

            % Saturazione della velocità richiesta
            obj.xdotbar = Saturate(qdot_ref, 0.5);
        end

        function updateJacobian(obj, robot)
            % Nell'Esercizio 3, ogni robot ha il proprio TPIK[cite: 95].
            % Il task agisce direttamente nello spazio dei giunti del singolo robot.
            % Pertanto, lo Jacobiano è l'identità 7x7.
            
            obj.J = eye(7); 
        end

        function updateActivation(obj, robot)
            % Calcolo della distanza minima dai limiti per ogni giunto
            dist_min = robot.q - robot.jlmin;
            dist_max = robot.jlmax - robot.q;
            
            % Prende la distanza minima (che sia verso il limite superiore o inferiore)
            dist_effettiva = min(dist_min, dist_max);
            
            % Attivazione: 1 al limite, 0 oltre il delta
            % Utilizza la funzione a campana decrescente per un'attivazione morbida
            A_diag = arrayfun(@(d) DecreasingBellShapedFunction(0, obj.delta, 0, 1, d), dist_effettiva);

            obj.A = diag(A_diag);
        end
    end
end