function main()
%Add path
addpath('./simulation_scripts');
addpath('./tools')
addpath('./icat')
addpath('./tasks')
clc; clear; close all;

%Simulation Parameters
dt = 0.005;
end_time = 10;

% Initialize Franka Emika Panda Model
model = load("panda.mat");

% Simulation Setup
real_robot = false;

% Initiliaze panda_arm() Class, specifying the base offset w.r.t World Frame
arm1=panda_arm(model,eye(4));
% TO DO: TRANSFORMATION MATRIX FROM WORLD FRAME TO RIGHT ARM BASE FRAME
wTb2 =[-1 0 0  1.06;
       0 -1 0  -0.01;
       0 0  1   0;
       0 0  0   1];
arm2=panda_arm(model,wTb2);

%Initialize Bimanual Simulator Class
bm_sim=bimanual_sim(dt,arm1,arm2,end_time);

%Define Object Shape and origin Frame
obj_length = 0.10;
w_obj_pos = [0.5 0 0.59]';
w_obj_ori = rotation(0,0,0);

%Set goal frames for left and right arm, based on object frame
%TO DO: Set arm goal frame based on object frame.
% Goal frames are at +-obj_length/2 (with an offset to take the obj not exactly at the extremes) along its x-axis
offset = (obj_length/2) - 0.01;
arm_dist_offset = [offset 0 0]';
arm1.setGoal(w_obj_pos, w_obj_ori, -arm_dist_offset, rotation(pi, -pi/6, 0));
% We take into account that the right arm is rotated of 180 degrees along z w.r.t left arm
arm2.setGoal(w_obj_pos, w_obj_ori, +arm_dist_offset, rotation(pi, -pi/6, pi));

%Define Object goal frame (Cooperative Motion)
wTog=[rotation(0,0,0) [0.65, -0.35, 0.28]'; 0 0 0 1];
arm1.set_obj_goal(wTog)
arm2.set_obj_goal(wTog)

%Define Tasks, input values(Robot type(L,R,BM), Task Name)
% Go to Position Tasks
left_tool_task=tool_task("L","LT");
right_tool_task=tool_task("R","RT");
tool_alt_task_L = ToolAltitudeLimit("L","TOOL_ALT_TASK_L");
tool_alt_task_R = ToolAltitudeLimit("R","TOOL_ALT_TASK_R");
jl_L = JointLimitTask('L', 'JL_Safe', 0.15);
jl_R = JointLimitTask("R","JR_Safe", 0.15);



object_task_l = ObjectTask("L","OBJECT_MOTION_L");
object_task_r = ObjectTask("R","OBJECT_MOTION_R");
rigid_constraint = RigidConstrTask("BM", "RIGID_CONSTRAINT");
stop_velocities_task_L = StopTask("L","STOP_VEL_L");
stop_velocities_task_R = StopTask("R","STOP_VEL_R");

%---------------------------------------------



% Bimanual Rigid Constraint Tasks


% Stop Motion Tasks


%Actions for each phase: go to phase, coop_motion phase, end_motion phase
go_to = {tool_alt_task_L, tool_alt_task_R, jl_L, jl_R, left_tool_task, right_tool_task};
bimanual_manipulation = {rigid_constraint, tool_alt_task_L, tool_alt_task_R, jl_L, jl_R, object_task_l, object_task_r};
stop_motion = {tool_alt_task_L, tool_alt_task_R, stop_velocities_task_R, stop_velocities_task_L};

% Unifying task list
unified_task_list = {rigid_constraint, tool_alt_task_L, tool_alt_task_R, jl_L, jl_R, left_tool_task, right_tool_task, object_task_l, object_task_r, stop_velocities_task_R, stop_velocities_task_L};
%Load Action Manager Class and load actions
actionManager = ActionManager();
actionManager.addAction(go_to, "Go To Position");
actionManager.addAction(bimanual_manipulation, "Bimanual Manipulation");
actionManager.addAction(stop_motion, "Stop Motion");
actionManager.addUnifyingTaskList(unified_task_list);

disp(actionManager.actionsName)

% Track mission phases
missionManager = MissionManager();
missionManager.missionPhase = 1;

%Initiliaze robot interface
robot_udp=UDP_interface(real_robot);

%Initialize logger
logger=SimulationLogger(ceil(end_time/dt)+1,bm_sim,actionManager);

%Main simulation Loop
for t = 0:dt:end_time
    % 1. Receive UDP packets - DO NOT EDIT
    [ql,qr]=robot_udp.udp_receive(t);
    if real_robot==true %Only in real setup, assign current robot configuration as initial configuratio
        bm_sim.left_arm.q=ql;
        bm_sim.right_arm.q=qr;
    end
    
    % 2. Update Full kinematics of the bimanual system
    bm_sim.update_full_kinematics();
  
    % Update Mission Phase based on current state
    missionManager.updateMissionPhase(actionManager, bm_sim);
    
    % 3. Compute control commands for current action
    [q_dot]=actionManager.computeICAT(bm_sim, dt);

    % 4. Step the simulator (integrate velocities)
    bm_sim.sim(q_dot);
    
    % 5. Send updated state to Pybullet
    robot_udp.send(t,bm_sim)
    
    % 6. Lggging
    logger.update(bm_sim.time,bm_sim.loopCounter)
    if mod(bm_sim.loopCounter, round(1 / bm_sim.dt)) == 0
        fprintf('t = %.2f s\n', bm_sim.time);
        if missionManager.missionPhase == 2
            fprintf('Left arm distance to goal = %.3f m\n', bm_sim.left_arm.dist_to_goal);
            fprintf('Right arm distance to goal = %.3f m\n', bm_sim.right_arm.dist_to_goal);
        end
            %fprintf("left arm altitude = %.3f m\n", bm_sim.left_arm.alt);
        %fprintf("right arm altitude = %.3f m\n", bm_sim.right_arm.alt);
    end

    % 7. Optional real-time slowdown
    SlowdownToRealtime(dt);
end
%Display joint position and velocity, Display for a given action, a number
%of tasks
action=2;
tasks=[1,5,6];
logger.plotAll(action,tasks);
end
