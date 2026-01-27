function main()
%Add path
addpath('./simulation_scripts');
addpath('./tools')
addpath('./icat')
addpath('./tasks')
clc;clear;close all; 
%Simulation Parameters
dt = 0.005;
end_time = 20;

% Initialize Franka Emika Panda Model
model = load("panda.mat");

%Simulation Setup
real_robot = false;

%Initiliaze panda_arm() Class, specifying the base offset w.r.t World Frame
arm1=panda_arm(model,eye(4));
%TO DO: TRANSFORMATION MATRIX FROM WORLD FRAME TO RIGHT ARM BASE FRAME
wTb2 =[-1 0 0  1.06;
       0 -1 0  -0.01;
       0 0  1   0;
       0 0  0   1];
arm2=panda_arm(model,wTb2);

%Initialize Cooperative Simulator Class
coop_system=coop_sim(dt,arm1,arm2,end_time);

%Define Object Shape and origin Frame
obj_length = 0.06;
w_obj_pos = [0.5 0 0.30]';
w_obj_ori = rotation(0,0,0);

%Set goal frames for left and right arm, based on object frame
%TO DO: Set arm goal frame based on object frame.
offset = (obj_length/2) - 0.005;
arm_dist_offset = [offset 0 0]';
arm1.setGoal(w_obj_pos, w_obj_ori, -arm_dist_offset, rotation(pi, -pi/9, 0));    
arm2.setGoal(w_obj_pos, w_obj_ori, +arm_dist_offset, rotation(pi, pi/9, 0)*rotation(0,0,pi));

    % Phase 2 Goal: Object Cooperative Goal
wTog = [rotation(0,0,0) [0.60, 0.40, 0.48]'; 0 0 0 1]; % Goal Esercizio 3
arm1.set_obj_goal(wTog);
arm2.set_obj_goal(wTog);


    % 1. Phase 1 Tasks: Move Tool to Grasp Point
    % Assumo tu abbia una classe CartesianTask standard. 
    % Se non l'hai, usa ObjectTask ma ricorda che calcola l'errore sull'Oggetto, non sul Tool.
    left_tool_task = tool_task("L", "CART_L"); 
    right_tool_task = tool_task("R", "CART_R");
    
    % 2. Safety Tasks
    ee_alt_L = ToolAltitudeLimit("L", "ALT_L");
    ee_alt_R = ToolAltitudeLimit("R", "ALT_R");
    
    jl_L = JointLimitTask("L", "JL_L");
    jl_R = JointLimitTask("R", "JL_R");
    
    % 3. Phase 2 Tasks: Object Motion (Non-Cooperative computation)
    object_task_L = ObjectTask("L", "OBJ_MOT_L");
    object_task_R = ObjectTask("R", "OBJ_MOT_R");
    
    % 4. Phase 2 Tasks: Cooperative Constraint (Tracks v_coop)
    coop_task_L = CooperativeRigidConstrTask("L", "COOP_CSTR_L");
    coop_task_R = CooperativeRigidConstrTask("R", "COOP_CSTR_R");
    
    % 5. Phase 3 Tasks: Stop
    stop_task_L = StopTask("L", "STOP_L");
    stop_task_R = StopTask("R", "STOP_R");

    % --- ACTION DEFINITIONS ---
    
    % Action 1: Reach Grasping Point (Non-Cooperative)
    go_to_left  = {ee_alt_L, jl_L, left_tool_task};
    go_to_right = {ee_alt_R, jl_R, right_tool_task};

    % Action 2 (Computation): Compute Non-Cooperative Object Velocity
    % Qui usiamo ObjectTask per calcolare cosa vorrebbe fare il robot se fosse solo
    coop_manipulation_L = {ee_alt_L, jl_L, object_task_L};
    coop_manipulation_R = {ee_alt_R, jl_R, object_task_R};

    % Action 3 (Execution): Stop
    stop_motion_L = {ee_alt_L, stop_task_L};
    stop_motion_R = {ee_alt_R, stop_task_R};

    % --- ACTION MANAGERS (NON-COOPERATIVE / COMPUTATION) ---
    actionManagerL = ActionManager();
    actionManagerL.addAction(go_to_left, "Go To Left");
    actionManagerL.addAction(coop_manipulation_L, "Coop Calc Left");
    actionManagerL.addAction(stop_motion_L, "Stop Left");

    actionManagerR = ActionManager();
    actionManagerR.addAction(go_to_right, "Go To Right");
    actionManagerR.addAction(coop_manipulation_R, "Coop Calc Right");
    actionManagerR.addAction(stop_motion_R, "Stop Right");

    unifiedTasksL = {ee_alt_L, jl_L, left_tool_task, object_task_L, stop_task_L};
    actionManagerL.addUnifyingTaskList(unifiedTasksL);

    unifiedTasksR = {ee_alt_R, jl_R, right_tool_task, object_task_R, stop_task_R};
    actionManagerR.addUnifyingTaskList(unifiedTasksR);

    % --- ACTION MANAGERS (COOPERATIVE EXECUTION) ---
    % Used in Phase 2 AFTER computing v_coop. 
    % Priority: 1. Cooperative Constraint (Motion) 2. Safety
    % NOTA: Rimosso ObjectTask qui, perché il movimento è imposto dal CoopTask tramite v_coop
    
    actionManagerL_coop = ActionManager();
    actionManagerL_coop.addAction({coop_task_L, ee_alt_L, jl_L}, "Coop Exec Left");
    actionManagerL_coop.addUnifyingTaskList({coop_task_L, ee_alt_L, jl_L});

    actionManagerR_coop = ActionManager();
    actionManagerR_coop.addAction({coop_task_R, ee_alt_R, jl_R}, "Coop Exec Right");
    actionManagerR_coop.addUnifyingTaskList({coop_task_R, ee_alt_R, jl_R});
    % Track mission phases
    missionManager = MissionManager();
    missionManager.missionPhase = 1;

    % Initial bias for cooperation weights
    mu0 = 0.2;

    % Initiliaze robot interface
    robot_udp = UDP_interface(real_robot);

    % Initialize logger
    logger_left = SimulationLogger(ceil(end_time/dt)+1, coop_system.left_arm);
    logger_right = SimulationLogger(ceil(end_time/dt)+1, coop_system.right_arm);
    %Main simulation Loop
    for t = 0:dt:end_time
        % 1. Receive UDP packets - DO NOT EDIT
        [ql, qr] = robot_udp.udp_receive(t);
        if real_robot==true %Only in real setup, assign current robot configuration as initial configuratio
            coop_system.left_arm.q=ql;
            coop_system.right_arm.q=qr;
        end
        
        % 2. Update Full kinematics of the bimanual system
        coop_system.update_full_kinematics();

         % 3. TO DO: compute the TPIK for each manipulator with your action
         % manager

        % Update mission phase (Your logic inside MissionManager)
        missionManager.updateMissionPhase(actionManagerL, actionManagerR, coop_system);

        % 3. Compute Non-Cooperative / Primary Action velocities
        % In Phase 1: Moves to Grasp
        % In Phase 2: Computes "Desired" velocity based on ObjectTask
        [ql_dot_nc] = actionManagerL.computeICAT(coop_system.left_arm, dt);
        [qr_dot_nc] = actionManagerR.computeICAT(coop_system.right_arm, dt);

        % --- DEBUG ---
        % if norm(ql_dot_nc) == 0
        %     disp('ql_dot_nc è ZERO. Controllo errori cartesiani...');
        %     % Stampa l'errore che vede il task
        %     err_L = norm(left_tool_task.xdotbar); 
        %     fprintf('Reference Task L: %f\n', err_L);
        % end
        % -----------------------------
        
        if missionManager.missionPhase == 1
            % Phase 1: Independent Approach
            ql_dot = ql_dot_nc;
            qr_dot = qr_dot_nc;
        
        elseif missionManager.missionPhase == 2
            % --- COOPERATIVE POLICY (Exercise 3) ---
            
            % A. Get Desired Object Velocity (Reference)
            % Nota: Assumiamo che compute_desired_refVelocity usi logicamente l'ObjectTask
            % o calcoliamo direttamente l'errore qui se serve.
            % Se l'ObjectTask ha già aggiornato 'xdotbar' dentro computeICAT, possiamo recuperarlo.
            % Per sicurezza, ricalcoliamo il twist desiderato dell'oggetto:
            [v_ang, v_lin] = CartError(coop_system.left_arm.wTog, coop_system.left_arm.wTo);
            xdot_ref = [v_ang; v_lin]; % Twist [omega; v]
            
            % Saturazione reference globale
            xdot_ref = Saturate(xdot_ref, 0.3);

            % B. Compute Non-Cooperative Cartesian Velocities at the Object Frame
            % Jacobian Object (Left) * qdot_nc (Left)
            % Dobbiamo usare lo Jacobiano proiettato sull'oggetto. 
            % Recuperiamolo dal task o ricalcoliamolo:
            J_oL = object_task_L.J; % J aggiornato nell'ultima iterazione
            x_dot_t_a = J_oL * ql_dot_nc;
            
            J_oR = object_task_R.J;
            x_dot_t_b = J_oR * qr_dot_nc;

            % C. Compute Weights (Coordination Policy)
            mu_a = mu0 + norm(xdot_ref - x_dot_t_a);
            mu_b = mu0 + norm(xdot_ref - x_dot_t_b);

            % D. Weighted Cooperative Velocity (xhat_dot)
            x_hat_dot = (mu_a*x_dot_t_a + mu_b*x_dot_t_b) / (mu_a + mu_b);

            % E. Rigid Grasp Constraints & Projection
            % [Ha, Hb, C] deve restituire le matrici per [omega; v] o [v; omega] consistenti
            [Ha, Hb, C] = computeRigidGraspConstraints(coop_system.left_arm, coop_system.right_arm);
            Hab = [Ha, zeros(6); zeros(6), Hb];

            % Project onto feasible subspace
            x_tilde_dot = Hab * (eye(12) - pinv(C)*C) * [x_hat_dot; x_hat_dot];

            % F. Set Cooperative Reference for the TASKS
            % CooperativeRigidConstrTask legge 'robot.v_coop'
            coop_system.left_arm.v_coop  = x_tilde_dot(1:6);
            coop_system.right_arm.v_coop = x_tilde_dot(7:12);
            
            % Passa anche la posa del partner per correzione drift (opzionale)
            coop_system.left_arm.wTo_partner = coop_system.right_arm.wTo;
            coop_system.right_arm.wTo_partner = coop_system.left_arm.wTo;

            % 4. Compute Cooperative Velocities
            % Ora usiamo i manager "Coop" che contengono CooperativeRigidConstrTask
            [ql_dot] = actionManagerL_coop.computeICAT(coop_system.left_arm, dt);
            [qr_dot] = actionManagerR_coop.computeICAT(coop_system.right_arm, dt);
        
        elseif missionManager.missionPhase == 3
            % Phase 3: Stop
            % Assicuriamoci che l'Action Manager abbia switchato all'azione "Stop"
            ql_dot = ql_dot_nc;
            qr_dot = qr_dot_nc;      
        end  

        % 6. get the two variables for integration
        coop_system.sim(ql_dot, qr_dot);
        
        % 6. Send updated state to Pybullet
        robot_udp.send(t, coop_system);

        % 7. Loggging
        logger_left.update(coop_system.time, coop_system.loopCounter);
        logger_right.update(coop_system.time, coop_system.loopCounter);
        
        if mod(coop_system.loopCounter, round(1 / coop_system.dt)) == 0
            fprintf('t = %.2f s, Phase: %d\n', coop_system.time, missionManager.missionPhase);        
        end
        
        SlowdownToRealtime(dt);
    end
    %9. Display joint position, velocity and end effector velocities, Display for a given action, a number
    %of tasks
    % action=1;
    % tasks=[1];
    logger_left.plotAll();
    logger_right.plotAll();
end