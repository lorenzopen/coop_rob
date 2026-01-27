classdef TaskProximityBoundary < Task
    properties

    end

    methods
        function updateReference(obj, robot)
            [~,lin] = CartError(robot.wTg, robot.wTv);

            obj.xdotbar = - 0.4 * lin(1:2);
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        function updateJacobian(obj, robot)
            Jt_a  = zeros(2,7);
            wRv = robot.wTv(1:3, 1:3);
            J_full_v = [(-wRv) zeros(3)];
            Jt_v = J_full_v(1:2, :);
            obj.J = [Jt_a Jt_v];
        end

        function updateActivation(obj, robot)
            [~,lin] = CartError(robot.wTg, robot.wTv);
            dist = norm(lin(1:2));
            obj.A = eye(2) * IncreasingBellShapedFunction(1.5,1.7,0,1,dist);
        end
    end
end

% classdef TaskProximityBoundary < Task
%     properties
%         % Flag per garantire che il reset avvenga una sola volta
%         goalReset = false; 
        
%         % Parametri per la logica di reset
%         rmax = 2.0;            % Raggio massimo (da impostare secondo necessità)
%         arm_goal = [10.5; 37.5];     % Posizione del target del braccio (x,y)
%     end

%     methods
%         function updateReference(obj, robot)
%             % 1. Recupera la posizione attuale del veicolo (x, y)
%             v_pos = robot.wTv(1:2, 4);
            
%             % 2. Calcola il vettore distanza verso il target del braccio (nodulo)
%             d_vec = obj.arm_goal(1:2) - v_pos;
%             d = norm(d_vec);

%             % 3. Logica di Reset: Esegui solo se la distanza è eccessiva e non è stato già fatto
%             if d > obj.rmax && ~obj.goalReset
%                 obj.goalReset = true;
                
%                 fprintf('Adjusting vehicle goal to guarantee nodule reachability (dist=%.2f)\n', d);
                
%                 % Calcola la correzione: ci avviciniamo lungo la linea d_vec
%                 % affinché la distanza finale sia rmax.
%                 correction = d_vec * (1 - obj.rmax/d);
                
%                 % Aggiorniamo il goal del veicolo (wTg)
%                 % Nota: Assumiamo che robot.wTg sia la matrice di trasformazione del goal
%                 robot.wTg(1:2, 4) = robot.wTg(1:2, 4) + correction;
                
%                 % Se il robot ha un metodo specifico 'setGoal', usalo qui. 
%                 % Altrimenti la modifica diretta a wTg (se robot è un handle) è sufficiente.
%             end

%             % 4. Calcolo standard dell'errore e della velocità di riferimento
%             [~, lin] = CartError(robot.wTg, robot.wTv);

%             obj.xdotbar = -0.4 * lin(1:2);
%             obj.xdotbar = Saturate(obj.xdotbar, 0.2);
%         end

%         function updateJacobian(obj, robot)
%             Jt_a  = zeros(2,7);
%             wRv = robot.wTv(1:3, 1:3);
%             J_full_v = [(-wRv) zeros(3)];
%             Jt_v = J_full_v(1:2, :);
%             obj.J = [Jt_a Jt_v];
%         end

%         function updateActivation(obj, robot)
%             [~,lin] = CartError(robot.wTg, robot.wTv);
%             dist = norm(lin(1:2));
%             % Attivazione basata sulla distanza
%             obj.A = eye(2) * IncreasingBellShapedFunction(1.5, 1.7, 0, 1, dist);
%         end
        
%         % Metodo opzionale per resettare il task se necessario riutilizzarlo
%         function resetState(obj)
%             obj.goalReset = false;
%         end
%     end
% end