classdef MissionManager < handle
    properties
        pos_threshold = 0.01;   % meters
        ang_threshold = 0.1;    % radians (approximately 5.7 degrees)
        % flags for phase transition
        phase_rigid_constraint_flag = false;
        phase_stop_motion_flag = false;

        missionPhase = 1; % 1=GoToPosition, 2=Bimanual Manipulation, 3=Stop Motion
    end

    methods

        function obj = MissionManager()
        end

        function updateMissionPhase(obj, actionManager, bm_sim)
            if obj.missionPhase == 1
                % PHASE 1: Reaching grasping points
                R1_error = bm_sim.left_arm.wTg(1:3, 1:3)' * bm_sim.left_arm.wTt(1:3, 1:3);
                R2_error = bm_sim.right_arm.wTg(1:3, 1:3)' * bm_sim.right_arm.wTt(1:3, 1:3);
                % Angular error
                angle_error_L = acos((trace(R1_error) - 1) / 2);
                angle_error_R = acos((trace(R2_error) - 1) / 2);

                % Linear errors
                arm1_pos_error = norm(bm_sim.left_arm.wTt(1:3,4) - bm_sim.left_arm.wTg(1:3,4));
                arm2_pos_error = norm(bm_sim.right_arm.wTt(1:3,4) - bm_sim.right_arm.wTg(1:3,4));

                if max(arm1_pos_error, arm2_pos_error) < obj.pos_threshold && ...
                        max(angle_error_L, angle_error_R) < obj.ang_threshold && ...
                        ~obj.phase_rigid_constraint_flag
                    disp("Arms reached grasping points!");
                    % TRANSITION TO PHASE 2: Grasping points reached
                    disp("Go To Position completed - Bimanual Manipulation starts");

                    % Define the object frame at grasping point
                    bm_sim.left_arm.compute_object_frame();
                    bm_sim.right_arm.compute_object_frame();

                    % Store object pose for reference
                    obj_world_pose = bm_sim.left_arm.wTo;
                    fprintf("Object frame at position: [%.3f, %.3f, %.3f]\n", ...
                        obj_world_pose(1,4), obj_world_pose(2,4), obj_world_pose(3,4));

                    fprintf("Object goal position set to: [%.3f, %.3f, %.3f]\n", ...
                        bm_sim.left_arm.wTog(1,4), bm_sim.left_arm.wTog(2,4), bm_sim.left_arm.wTog(3,4));

                    % Set binary transition (no smoothness) for rigid constraint
                    actionManager.setBinaryTransition(true);
                    actionManager.setCurrentAction("Bimanual Manipulation");
                    obj.phase_rigid_constraint_flag = true;
                    obj.missionPhase = 2;
                end
            elseif obj.missionPhase == 2
                % PHASE 2: Rigid body grasping and object movement
                % Check if object reached goal position
                object_pos_error = norm(bm_sim.left_arm.wTo(1:3,4) - bm_sim.left_arm.wTog(1:3,4));
                %fprintf("Object position error: %.3f\n", object_pos_error);
                if object_pos_error < obj.pos_threshold && ~obj.phase_stop_motion_flag
                    disp("Object reached goal position!");
                    fprintf("Final object position: [%.3f, %.3f, %.3f]\n", ...
                        bm_sim.left_arm.wTo(1,4), bm_sim.left_arm.wTo(2,4), bm_sim.left_arm.wTo(3,4));

                    disp("Bimanual Manipulation completed - Stop Motion starts");
                    
                    actionManager.setBinaryTransition(false);
                    actionManager.setCurrentAction("Stop Motion");
                    obj.phase_stop_motion_flag = true;       
                    obj.missionPhase = 3;             
                end
            end
        end
    end
end