classdef ee_min_altitude_task < Task
    properties

    end

    methods
        function obj=ee_min_altitude_task(robot_ID,taskID)
            obj.ID=robot_ID;
            obj.task_name=taskID;
        end

        function updateReference(obj, robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end
            
            error = 0.15 - robot.alt; % minimum altitude = 0.15m
            obj.xdotbar = 1.0 * error;         
            % limit the requested velocities...
            obj.xdotbar = Saturate(obj.xdotbar, 1.0);
        end

        function updateJacobian(obj,robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end
            tool_jacobian_lin=robot.wJt(1:3,:);
            
            if obj.ID=='L'
                obj.J=[0 0 1] * [tool_jacobian_lin, zeros(3, 7)];
            elseif obj.ID=='R'
                obj.J=[0 0 1] * [zeros(3, 7), tool_jacobian_lin];
            end
        end

        function updateActivation(obj, robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end

            alt = robot.alt;
            obj.A = DecreasingBellShapedFunction(0.15, 0.2, 0, 1, alt);
        end
    end
end