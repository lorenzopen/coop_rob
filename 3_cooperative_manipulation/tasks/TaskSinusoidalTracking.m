classdef TaskSinusoidalTracking < Task
    properties
        wTg_0       % Posa nominale centrale dell'oscillazione
        amplitude   % Ampiezza (m)
        frequency   % Frequenza (rad/s)
        t_internal  % Timer
        Kp = 0.3;   % Guadagno proporzionale
    end
 
    methods
        function obj = TaskSinusoidalTracking(robID, taskID, wTg_init, amp, freq)
            obj.ID = robID;
            obj.task_name = char(taskID);
            obj.wTg_0 = wTg_init;
            obj.amplitude = amp;
            obj.frequency = freq;
            obj.t_internal = 0;
        end
        function updateReference(obj, robot)
            dt = 0.005; % Uguale al dt del main
            obj.t_internal = obj.t_internal + dt;
            wTg_des = obj.wTg_0;
            wTg_des(3,4) = obj.wTg_0(3,4) + obj.amplitude * sin(obj.frequency * obj.t_internal); %cosa fa esattamente? non è che sovrascrive goal? perche scendono dritti troppo e poi oscillano
            z_dot_ff = obj.amplitude * obj.frequency * cos(obj.frequency * obj.t_internal);
            v_ff = [0; 0; 0; 0; 0; z_dot_ff]; 
            [err_ang, err_lin] = CartError(wTg_des, robot.wTt);
            obj.xdotbar = v_ff + obj.Kp * [err_ang; err_lin];
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.3);
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.3);
        end
 
        function updateJacobian(obj, robot)
            obj.J = robot.wJt;
        end
 
        function updateActivation(obj, robot)
            obj.A = eye(6);
        end
    end
end