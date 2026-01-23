function main()
%Add path
addpath('./simulation_scripts');
addpath('./tools')
addpath('./icat')
addpath('./tasks')
clc;clear;close all; 
%Simulation Parameters
dt = 0.005;
end_time = 10;

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

%Define Object goal frame (Cooperative Motion)
wTog=[rotation(0,0,0) [0.6, 0.4, 0.48]'; 0 0 0 1];
arm1.set_obj_goal(wTog)
arm2.set_obj_goal(wTog)

%Define Tasks, input values(Robot type(L,R,BM), Task Name)
left_tool_task=tool_task("L","LT");
right_tool_task=tool_task("R","RT");
ee_alt_L = ee_min_altitude_task("L","EE_ALT_L");
ee_alt_R = ee_min_altitude_task("R","EE_ALT_R");
jl_L = joint_limits_task("L","JL_L");
jl_R = joint_limits_task("R","JL_R");
object_task_L = object_motion_task("L","OBJ_MOTION_L");
object_task_R = object_motion_task("R","OBJ_MOTION_R");
coop_tool_velocity_L = coop_tool_velocity_task("L","COOP_TOOL_VEL_L");
coop_tool_velocity_R = coop_tool_velocity_task("R","COOP_TOOL_VEL_R");
stop_velocities_task_L = stop_velocities_task("L","STOP_VEL_L");
stop_velocities_task_R = stop_velocities_task("R","STOP_VEL_R");

%TO DO: Define the actions for each manipulator (remember the specific one
%for the cooperation)
go_to_left={ee_alt_L, jl_L, left_tool_task};
go_to_right={ee_alt_R, jl_R, right_tool_task};

coop_manipulation_L={ee_alt_L, jl_L, object_task_L};
coop_manipulation_R={ee_alt_R, jl_R, object_task_R};

stop_motion_L = {ee_alt_L, stop_velocities_task_L};
stop_motion_R = {ee_alt_R, stop_velocities_task_R};

% Unified Lists 
% forse non servono dato che gestiamo le azioni singolarmente
% e c'è distinzione parte non cooperativa e cooperativa
unifiedTasksL={ee_alt_L, jl_L, left_tool_task, object_task_L, stop_velocities_task_L};
unifiedTasksR={ee_alt_R, jl_R, right_tool_task, object_task_R, stop_velocities_task_R};

%TO DO: Create two action manager objects to manage the tasks of a single
%manipulator (one for the non-cooperative and one for the cooperative steps
%of the algorithm)
actionManagerL = ActionManager();
actionManagerL.addAction(go_to_left, "Go To Left");
actionManagerL.addAction(coop_manipulation_L, "Cooperative Manipulation Left");
actionManagerL.addAction(stop_motion_L, "Stop Motion left");
actionManagerL.addUnifyingTaskList(unifiedTasksL);
disp('Left Action Manager actions:');
disp(actionManagerL.actionsName);

actionManagerR = ActionManager();
actionManagerR.addAction(go_to_right, "Go To Right");
actionManagerR.addAction(coop_manipulation_R, "Cooperative Manipulation Right");
actionManagerR.addAction(stop_motion_R, "Stop Motion right");
actionManagerR.addUnifyingTaskList(unifiedTasksR);
disp('Right Action Manager actions:');
disp(actionManagerR.actionsName);

% cooperative action manager for both arms during Cooperative Manipulation action
% same tasks but with the cooperative tool velocity task with top priority
actionManagerL_coop = ActionManager();
actionManagerL_coop.addAction({coop_tool_velocity_L, ee_alt_L, jl_L, object_task_L}, "feasible vel Left");
actionManagerL_coop.addUnifyingTaskList({coop_tool_velocity_L, ee_alt_L, jl_L, object_task_L});

actionManagerR_coop = ActionManager();
actionManagerR_coop.addAction({coop_tool_velocity_R, ee_alt_R, jl_R, object_task_R}, "feasible vel Right");
actionManagerR_coop.addUnifyingTaskList({coop_tool_velocity_R, ee_alt_R, jl_R, object_task_R});

% Track mission phases
missionManager = MissionManager();
missionManager.missionPhase = 1;

% Initial bias for cooperation weights
mu0 = 0.2;

%Initiliaze robot interface
robot_udp=UDP_interface(real_robot);

%Initialize logger
logger_left=SimulationLogger(ceil(end_time/dt)+1,coop_system.left_arm);
logger_right=SimulationLogger(ceil(end_time/dt)+1,coop_system.right_arm);
%Main simulation Loop
for t = 0:dt:end_time
    % 1. Receive UDP packets - DO NOT EDIT
    [ql,qr]=robot_udp.udp_receive(t);
    if real_robot==true %Only in real setup, assign current robot configuration as initial configuratio
        coop_system.left_arm.q=ql;
        coop_system.right_arm.q=qr;
    end
    % 2. Update Full kinematics of the bimanual system
    coop_system.update_full_kinematics();

    % Update mission phase based on current actions and robot/object states
    missionManager.updateMissionPhase(actionManagerL, actionManagerR, coop_system);

    % 3. TO DO: compute the TPIK for each manipulator with your action
    % manager
    [ql_dot_nc]=actionManagerL.computeICAT(coop_system.left_arm, dt);
    [qr_dot_nc]=actionManagerR.computeICAT(coop_system.right_arm, dt);
    
    if missionManager.missionPhase == 1
        % Phase 1: just use non-cooperative velocities
        ql_dot = ql_dot_nc;
        qr_dot = qr_dot_nc;
    
    elseif missionManager.missionPhase == 2
        % RIGID BODY CONSTRAINT
        % Reference generator: desired object twist

        % 4. TO DO: COOPERATION hierarchy
        % SAVE THE NON COOPERATIVE VELOCITIES COMPUTED
        xdot_ref = coop_system.left_arm.compute_desired_refVelocity();

        % Share reference with both arms
        coop_system.right_arm.xdot = xdot_ref;

        % Compute non-cooperative tool velocities
        x_dot_t_a = coop_system.left_arm.wJt * ql_dot_nc;
        x_dot_t_b = coop_system.right_arm.wJt * qr_dot_nc;

        % Compute weights        
        mu_a = mu0 + norm(xdot_ref - x_dot_t_a);
        mu_b = mu0 + norm(xdot_ref - x_dot_t_b);

        % Weighted cooperative velocity (xhat_dot)
        x_hat_dot = (mu_a*x_dot_t_a + mu_b*x_dot_t_b) / (mu_a + mu_b);

        % Compute rigid grasp constraint and feasible space
        [Ha,Hb,C] = computeRigidGraspConstraints(coop_system.left_arm, coop_system.right_arm);
        Hab = [Ha zeros(6)
              zeros(6) Hb];

        % Project onto feasible subspace (avoid internal forces)
        x_tilde_dot = Hab * (eye(12) - pinv(C)*C) * [x_hat_dot; x_hat_dot];

        % Set cooperative task reference
        coop_system.left_arm.xdot_coop  = x_tilde_dot(1:6);
        coop_system.right_arm.xdot_coop = x_tilde_dot(7:12);

        % 5. TO DO: compute the TPIK for each manipulator with your action
        % manager (with the constrained action to track the coop velocity)
        [ql_dot] = actionManagerL_coop.computeICAT(coop_system.left_arm, dt);
        [qr_dot] = actionManagerR_coop.computeICAT(coop_system.right_arm, dt);
    
    elseif missionManager.missionPhase == 3
        % Phase 3: just use non-cooperative velocities
        ql_dot = ql_dot_nc;
        qr_dot = qr_dot_nc;      
    end  

    % 6. get the two variables for integration
    coop_system.sim(ql_dot,qr_dot);
    
    % 6. Send updated state to Pybullet
    robot_udp.send(t,coop_system)

    % 7. Loggging
    logger_left.update(coop_system.time,coop_system.loopCounter)
    logger_right.update(coop_system.time,coop_system.loopCounter)
    if mod(coop_system.loopCounter, round(1 / coop_system.dt)) == 0
        fprintf('t = %.2f s\n', coop_system.time);        
    end
    % 8. Optional real-time slowdown
    SlowdownToRealtime(dt);
end
%9. Display joint position, velocity and end effector velocities, Display for a given action, a number
%of tasks
% action=1;
% tasks=[1];
logger_left.plotAll();
logger_right.plotAll();

end
