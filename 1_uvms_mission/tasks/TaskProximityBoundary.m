classdef TaskProximityBoundary < Task
    properties
        % Definiamo i margini operativi sicuri per il braccio
        R_min = 0.8; % Distanza minima dal target (troppo vicino)
        R_max = 1.3; % Distanza massima dal target (troppo lontano)
    end

    methods
        function updateReference(obj, robot)
            [~,v_lin] = CartError(robot.wTg, robot.wTv);
            dist = norm(v_lin(1:2));
            
            % Calcoliamo il versore che punta dal veicolo verso il target
            if dist > 0
                u_dir = v_lin(1:2) / dist; 
            else
                u_dir = [0; 0];
            end
            
            % Determiniamo l'errore di distanza
            if dist < obj.R_min
                % Troppo vicino: errore negativo -> la velocità farà allontanare il veicolo
                err = dist - obj.R_min; 
            elseif dist > obj.R_max
                % Troppo lontano: errore positivo -> la velocità farà avvicinare il veicolo
                err = dist - obj.R_max; 
            else
                % Nella zona sicura l'errore è nullo
                err = 0; 
            end
            
            % Calcolo della velocità di riferimento (xdotbar)
            % Se err < 0 (troppo vicino), il vettore punta in direzione opposta al target
            obj.xdotbar = 0.5 * err * u_dir; 
            
            % Saturazione per sicurezza
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        
        function updateJacobian(obj, robot)
            Jt_a  = zeros(2,7);
            J_v = [-robot.wTv(1:2, 1:3) zeros(2,3)];
            obj.J = [Jt_a J_v];
        end

        function updateActivation(obj, robot)
            [~,v_lin] = CartError(robot.wTg, robot.wTv);
            dist = norm(v_lin(1:2));
            
            % 1. Attivazione limite interno (si attiva quando si è troppo vicini)
            % Passa da 0 a 1 quando la distanza scende sotto R_min
            A_inner = DecreasingBellShapedFunction(obj.R_min - 0.2, obj.R_min, 0, 1, dist);
            
            % 2. Attivazione limite esterno (si attiva quando si è troppo lontani)
            % Passa da 0 a 1 quando la distanza sale sopra R_max
            A_outer = IncreasingBellShapedFunction(obj.R_max, obj.R_max + 0.2, 0, 1, dist);
            
            % Il task si attiva se viene violato uno qualsiasi dei due limiti (max tra i due)
            obj.A = eye(2) * max(A_inner, A_outer);
        end
    end
end