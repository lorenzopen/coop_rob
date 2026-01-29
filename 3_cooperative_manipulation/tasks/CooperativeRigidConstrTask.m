classdef CooperativeRigidConstrTask < Task
    % Implementazione basata su CoopObjectTask del codice "Friend"
    % Calcola la velocità di riferimento proiettando la velocità cooperativa
    % desiderata (xdot_hat) sul vincolo di presa rigida (C).

    properties
        H_lr       % Matrice combinata degli spazi di moto
        C          % Matrice del vincolo [Ha, -Hb]
        xdot_hat   % Velocità cooperativa pesata (calcolata nel main)
    end

    methods
        function obj = CooperativeRigidConstrTask(robotID, taskID)
            obj.ID = robotID;
            obj.task_name = taskID;
            %obj.kp = 1.0; 
        end
        
        function updateReference(obj, robot)
            % 1. Proiezione sul vincolo
            % Formula: v_constrained = H_lr * (I - pinv(C)*C) * [v_coop; v_coop]
            % Questo assicura che le velocità siano cinematicamente consistenti
            
            if isempty(obj.C) || isempty(obj.H_lr)
                obj.xdotbar = zeros(6,1);
                return;
            end

            % Vettore velocità cooperativa ripetuto per i due bracci
            v_stacked = [obj.xdot_hat; obj.xdot_hat];

            % Proiettore nel nullo del vincolo
            P_constr = (eye(12) - pinv(obj.C) * obj.C);

            % Calcolo velocità ammissibile nel frame dell'oggetto
            xdot_tilde = obj.H_lr * P_constr * v_stacked;

            % 2. Assegnazione al robot corretto
            if strcmp(obj.ID, 'L')
                obj.xdotbar = xdot_tilde(1:6);
            elseif strcmp(obj.ID, 'R')
                obj.xdotbar = xdot_tilde(7:12);    
            end

            % Saturazione di sicurezza
            obj.xdotbar = Saturate(obj.xdotbar, 0.25);
        end
        
        function updateJacobian(obj, robot)        
            % Usa lo Jacobiano dell'oggetto (wJo)
            % Assicurati che la classe panda_arm calcoli wJo correttamente
            
            if isprop(robot, 'wJo') && ~isempty(robot.wJo)
                obj.J = robot.wJo;
            else
                % Fallback se wJo non è calcolato in panda_arm, 
                % calcolalo qui trasportando wJt sull'oggetto
                r_to = robot.wTo(1:3, 4) - robot.wTt(1:3, 4);
                S_r = skew(r_to);
                X_to = [eye(3), zeros(3); -S_r, eye(3)]; % Twist [omega; v] convention?
                % VERIFICA LA TUA CONVENZIONE: Il codice del tuo amico usa 
                % wJo direttamente. Assumiamo che panda_arm lo abbia.
                obj.J = X_to * robot.wJt; 
            end
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(6);
        end
    end
end