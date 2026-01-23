classdef TaskOrient < Task   
    properties
        error % scalare per il logging
    end

    methods
        function updateReference(obj, robot)
            % Calcola l'errore di orientamento tra il Goal (wTgv) e il Corrente (wTv)
            % CartError restituisce il vettore errore angolare (asse * angolo)
            [w_ang, ~] = CartError(robot.wTgv, robot.wTv);
            
            % Guadagno proporzionale (puoi alzarlo a 0.5 se vuoi più reattività)
            obj.xdotbar = 0.5 * w_ang;
            
            % Saturazione per sicurezza
            obj.xdotbar = Saturate(obj.xdotbar, 0.4);
            
            % Salviamo la norma dell'errore per i grafici/log
            obj.error = norm(w_ang);
        end

        function updateJacobian(obj, robot)
            % Dobbiamo mappare le velocità angolari del corpo (v_nu angolare) 
            % sulle velocità angolari del mondo (dove è calcolato l'errore).
            % Relazione: w_world = wRv * w_body
            
            wRv = robot.wTv(1:3, 1:3);
            
            % J = [zeros(3,7_giunti), zeros(3,3_vel_lin), wRv(3x3_vel_ang)]
            obj.J = [zeros(3,7), zeros(3,3), wRv];
        end
        
        function updateActivation(obj, robot)
            % Attivazione costante su tutti e 3 gli assi (Roll, Pitch, Yaw)
            obj.A = eye(3);
        end
    end
end