classdef ToolAltitudeLimit < Task
    % ToolAltitudeLimit (Decentralized)
    % Mantiene l'altitudine minima del Tool per il SINGOLO robot.
    
    properties
        z_min = 0.15;       % Limit inf (m)
        z_buff = 0.05;      % Buffer (m)
        gain = 1.0;         % gain
        smooth_active = true;
    end

    methods
        function obj = ToolAltitudeLimit(robID, taskID, smooth_flag)
            obj.ID = robID;
            obj.task_name = taskID;
            if nargin >= 3
                obj.smooth_active = smooth_flag;
            end
        end

        function updateReference(obj, robot)            
            % Lettura Altitudine Tool (Componente Z della posizione)
            % wTt è 4x4, l'elemento (3,4) è la z traslazionale.
            curr_alt = robot.wTt(3,4);

            if isscalar(curr_alt)
                % Calcolo errore: positivo se siamo sotto il limite (es. 0.15 - 0.10 = +0.05)
                err = obj.z_min - curr_alt;
                
                % agiamo solo se l'errore è positivo 
                % Il gain positivo genera una velocità positiva (+Z) per risalire.
                raw_vel = obj.gain * max(0, err);
                
                obj.xdotbar = raw_vel;
            else
                obj.xdotbar = 0;
            end

            % Saturazione di sicurezza (m/s)
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end

        function updateJacobian(obj, robot)
            
            % Jacobiano geometrico del tool (6x7)
            J_tool = robot.wJt;

            % 
            select_vec = [0 0 0 0 0 1];

            % Proiezione del Jacobiano sulla sola riga Z (risultato 1x7)
            
            obj.J = select_vec * J_tool;
        end

        function updateActivation(obj, robot)
            curr_alt = robot.wTt(3,4);

            % Attivazione soft:
            % 1 ( activation) se siamo under z_min
            % 0 (disabled) se siamo over z_min + buffer
            if isscalar(curr_alt) && obj.smooth_active
                obj.A = DecreasingBellShapedFunction(obj.z_min, ...
                                                     obj.z_min + obj.z_buff, ...
                                                     0, 1, curr_alt);
            elseif isscalar(curr_alt) && ~obj.smooth_active
                 % Attivazione hard (binaria) 
                 if curr_alt < obj.z_min
                     obj.A = 1;
                 else
                     obj.A = 0;
                 end
            else
                obj.A = 0;
            end
        end
    end
end