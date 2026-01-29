classdef CooperativeRigidConstrTask < Task

    properties
        H_lr       % Matrice combinata degli spazi di moto
        C          % [Ha, -Hb]
        xdot_hat   
    end

    methods
        function obj = CooperativeRigidConstrTask(robotID, taskID)
            obj.ID = robotID;
            obj.task_name = taskID;
            %obj.kp = 1.0; 
        end
        
        function updateReference(obj, robot)
            % 1. Proiezione 
            %  v_constrained = H_lr * (I - pinv(C)*C) * [v_coop; v_coop]
            
            
            if isempty(obj.C) || isempty(obj.H_lr)
                obj.xdotbar = zeros(6,1);
                return;
            end

            v_stacked = [obj.xdot_hat; obj.xdot_hat];

            % Proiettore nel nullo del vincolo
            P_constr = (eye(12) - pinv(obj.C) * obj.C);

            % Calcolo velocità ammissibile nel frame dell'oggetto
            xdot_tilde = obj.H_lr * P_constr * v_stacked;

            
            if strcmp(obj.ID, 'L')
                obj.xdotbar = xdot_tilde(1:6);
            elseif strcmp(obj.ID, 'R')
                obj.xdotbar = xdot_tilde(7:12);    
            end

            % Saturazione di sicurezza
            obj.xdotbar = Saturate(obj.xdotbar, 0.25);
        end
        
        function updateJacobian(obj, robot)        
            %  Jacobiano dell'oggetto (wJo)
            
            if isprop(robot, 'wJo') && ~isempty(robot.wJo)
                obj.J = robot.wJo;
            else
                % Fallback 
               
                r_to = robot.wTo(1:3, 4) - robot.wTt(1:3, 4);
                S_r = skew(r_to);
                X_to = [eye(3), zeros(3); -S_r, eye(3)]; 
                
                obj.J = X_to * robot.wJt; 
            end
        end
        
        function updateActivation(obj, robot)
            obj.A = eye(6);
        end
    end
end