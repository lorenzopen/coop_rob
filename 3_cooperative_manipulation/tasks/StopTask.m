classdef StopTask < Task
    % StopTask per Esercizio 3 (Decentralized)
    % Impone velocità zero a tutti i giunti del SINGOLO robot.
    % Usato nella fase finale per "congelare" il robot.
    
    properties
        % Nessuna proprietà specifica necessaria
    end
    
    methods
        function obj = StopTask(robot_ID, taskID)
            obj.ID = robot_ID;
            obj.task_name = taskID;
        end
        
        function updateReference(obj, robot)
            % INPUT: 'robot' è la struttura del SINGOLO braccio (7 DOF)
            
            % L'obiettivo è fermare i giunti: velocità desiderata = 0
            obj.xdotbar = zeros(7, 1);
            
            % NOTA: Se volessi mantenere la "frenata morbida" (damping) 
            % usata nel codice precedente, potresti fare:
            % obj.xdotbar = -0.5 * robot.qdot; 
            % Tuttavia, impostare xdotbar a 0 è il modo standard per
            % definire un task di arresto nel Task Priority.
        end
        
        function updateJacobian(obj, robot)
            % Il task controlla direttamente lo spazio dei giunti del robot.
            % In un sistema decentralizzato (7 DOF), lo Jacobiano è l'identità 7x7.
            
            obj.J = eye(7);
        end
        
        function updateActivation(obj, robot)
            % Attivazione totale su tutti i giunti
            obj.A = eye(7);
        end
    end
end