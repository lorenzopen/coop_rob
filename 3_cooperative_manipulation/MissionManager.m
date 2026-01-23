classdef MissionManager < handle
    properties
        pos_threshold = 0.01;   % meters
        ang_threshold = 0.1;    % radians
        % flags for phase transition
        phase_rigid_constraint_flag = false;
        phase_stop_motion_flag = false;

        missionPhase = 1; % 1=GoTo, 2=CooperativeManipulation, 3=StopMotion
    end

    methods

        function obj = MissionManager()
        end

        function updateMissionPhase(obj, actionManagerL, actionManagerR, coop_system)
            if obj.missionPhase == 1              
                % PHASE 1: Reaching grasping points                
                R1_error = coop_system.left_arm.wTg(1:3, 1:3)' * coop_system.left_arm.wTt(1:3, 1:3);
                R2_error = coop_system.right_arm.wTg(1:3, 1:3)' * coop_system.right_arm.wTt(1:3, 1:3);
                % Angular error
                angle_error_L = acos((trace(R1_error) - 1) / 2);
                angle_error_R = acos((trace(R2_error) - 1) / 2);

                % Linear errors
                arm1_pos_error = norm(coop_system.left_arm.wTt(1:3,4) - coop_system.left_arm.wTg(1:3,4));
                arm2_pos_error = norm(coop_system.right_arm.wTt(1:3,4) - coop_system.right_arm.wTg(1:3,4));

                if max(arm1_pos_error, arm2_pos_error) < obj.pos_threshold && ...
                        max(angle_error_L, angle_error_R) < obj.ang_threshold && ...
                        ~obj.phase_rigid_constraint_flag
                    disp("Arms reached grasping points!");
                    % TRANSITION TO PHASE 2: Grasping points reached
                    disp("Go To Position completed - Cooperative Manipulation starts");

                    % Define the object frame at grasping point
                    coop_system.left_arm.compute_object_frame();
                    coop_system.right_arm.compute_object_frame();

                    % Store object pose for reference
                    obj_world_pose = coop_system.left_arm.wTo;
                    fprintf("Object frame at position: [%.3f, %.3f, %.3f]\n", ...
                        obj_world_pose(1,4), obj_world_pose(2,4), obj_world_pose(3,4));

                    fprintf("Object goal position set to: [%.3f, %.3f, %.3f]\n", ...
                        coop_system.left_arm.wTog(1,4), coop_system.left_arm.wTog(2,4), coop_system.left_arm.wTog(3,4));

                    % Set binary transition (no smoothness) for rigid constraint
                    actionManagerL.setBinaryTransition(true);
                    actionManagerL.setCurrentAction("Cooperative Manipulation Left");
                    actionManagerR.setBinaryTransition(true);
                    actionManagerR.setCurrentAction("Cooperative Manipulation Right");
                    obj.phase_rigid_constraint_flag = true;
                    obj.missionPhase = 2;
                end
            elseif obj.missionPhase == 2
                % PHASE 2: Rigid body grasping and object movement
                % Check if object reached goal position
                object_pos_error = norm(coop_system.left_arm.wTo(1:3,4) - coop_system.left_arm.wTog(1:3,4));
                %fprintf("Object position error: %.3f\n", object_pos_error); % debug print
                if object_pos_error < obj.pos_threshold && ~obj.phase_stop_motion_flag
                    disp("Object reached goal position!");
                    fprintf("Final object position: [%.3f, %.3f, %.3f]\n", ...
                        coop_system.left_arm.wTo(1,4), coop_system.left_arm.wTo(2,4), coop_system.left_arm.wTo(3,4));

                    disp("Cooperative Manipulation completed - Stop Motion starts");
                    
                    actionManagerL.setBinaryTransition(false);
                    actionManagerR.setBinaryTransition(false);
                    actionManagerL.setCurrentAction("Stop Motion left");
                    actionManagerR.setCurrentAction("Stop Motion right");
                    obj.phase_stop_motion_flag = true;
                    obj.missionPhase = 3;
                end
            end
        end
    end
end