classdef JointLimitTask < Task
    % Task per l'evitamento dei limiti di giunto con attivazione morbida
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

        function updateReference(obj, robot_system)
            % Selezione del braccio direttamente qui
            if obj.ID == 'L'
                robot = robot_system.left_arm;
            elseif obj.ID == 'R'
                robot = robot_system.right_arm;
            else
                robot = robot_system;
            end
            
            % Calcolo dell'errore rispetto ai limiti (vettoriale)
            % err_min è positivo se il giunto scende sotto (jlmin + delta)
            % err_max è positivo se il giunto sale sopra (jlmax - delta)
            err_min = (robot.jlmin + obj.delta) - robot.q;
            err_max = robot.q - (robot.jlmax - obj.delta);

            % Calcoliamo la velocità di riferimento: spinge indietro solo se supera la soglia
            % Se err < 0, il giunto è in zona sicura -> qdot = 0
            qdot_ref = obj.gain * (max(err_min, 0) - max(err_max, 0));

            % Saturazione della velocità richiesta
            obj.xdotbar = Saturate(qdot_ref, 0.5);
        end

        function updateJacobian(obj, robot_system)
            % Supporta sia robot singoli (7 DOF) che dual-arm (14 DOF)
            I7 = eye(7);
            if obj.ID == 'L'
                % Se siamo il braccio sinistro, occupiamo le prime 7 colonne
                % Assumiamo che il braccio destro abbia dimensione pari alle righe di q
                obj.J = [I7, zeros(7, size(robot_system.right_arm.q, 1))];
            elseif obj.ID == 'R'
                % Se siamo il braccio destro, occupiamo le ultime 7 colonne
                obj.J = [zeros(7, size(robot_system.left_arm.q, 1)), I7];
            else
                obj.J = I7; % Caso braccio singolo
            end
        end

        function updateActivation(obj, robot_system)
            % Selezione del braccio direttamente qui
            if obj.ID == 'L'
                robot = robot_system.left_arm;
            elseif obj.ID == 'R'
                robot = robot_system.right_arm;
            else
                robot = robot_system;
            end
            
            % Calcolo della distanza minima dai limiti per ogni giunto
            dist_min = robot.q - robot.jlmin;
            dist_max = robot.jlmax - robot.q;
            
            % Attivazione: 1 al limite, 0 oltre il delta (zona sicura)
            % Usiamo arrayfun per applicare la funzione di attivazione a ogni giunto
            % Prende la distanza minima (che sia verso il limite superiore o inferiore)
            dist_effettiva = min(dist_min, dist_max);
            
            A_diag = arrayfun(@(d) DecreasingBellShapedFunction(0, obj.delta, 0, 1, d), dist_effettiva);

            obj.A = diag(A_diag);
        end
    end
end