classdef ActionManager < handle
    properties
        actions = {}      % cell array of actions (each action = stack of tasks)
        currentAction = 1 % index of currently active action
        
        
        previousAction = 1
        unifiedList = {}  % Global list defining priority order
        t = 0             % Transition timer
        T_trans = 2.0     % Transition duration [s]
        k_in = 1          % Activation scalar for incoming action
        k_out = 0         % Activation scalar for outgoing action
    end

    methods
        function addAction(obj, taskStack)
            % taskStack: cell array of tasks that define an action
            obj.actions{end+1} = taskStack;
        end

        function addUnifiedList(obj, list)
            % Necessary helper to define global priority order
            obj.unifiedList = list;
        end

        function [v_nu, qdot] = computeICAT(obj, robot, dt)
            % Update timer
            obj.t = obj.t + dt;

            % 1. Update references, Jacobians, activations for ALL tasks
            % We iterate on unifiedList to ensure complete system update
            for i = 1:length(obj.unifiedList)
                obj.unifiedList{i}.updateReference(robot);
                obj.unifiedList{i}.updateJacobian(robot);
                obj.unifiedList{i}.updateActivation(robot);
            end

            % 2. Compute transition scalars (Global)
            if obj.currentAction ~= obj.previousAction && obj.t <= obj.T_trans
                obj.k_in = IncreasingBellShapedFunction(0, obj.T_trans, 0, 1, obj.t);
                obj.k_out = DecreasingBellShapedFunction(0, obj.T_trans, 0, 1, obj.t);
            else
                % Steady state
                obj.k_in = 1; 
                obj.k_out = 0;
            end

            % Retrieve stacks
            currTasks = obj.actions{obj.currentAction};
            prevTasks = obj.actions{obj.previousAction};

            % 3. Perform ICAT (task-priority inverse kinematics)
            ydotbar = zeros(13,1);
            Qp = eye(13);
            
            % Iterate via Unified List to respect Global Priority
            for i = 1:length(obj.unifiedList)
                task = obj.unifiedList{i};

                % Check membership using handle comparison
                inCurr = any(cellfun(@(x) x == task, currTasks));
                inPrev = any(cellfun(@(x) x == task, prevTasks));
                
                % Determine activation weight
                w = 0;
                if inCurr && inPrev
                    w = 1;           % Persistent task
                elseif inCurr
                    w = obj.k_in;    % Incoming task
                elseif inPrev
                    w = obj.k_out;   % Outgoing task
                end

                % Execute iCAT only if task is active
                if w > 1e-6
                    [Qp, ydotbar] = iCAT_task(task.A * w, task.J, Qp, ydotbar, task.xdotbar, 1e-4, 0.01, 10);
                end
            end

            % 4. Last task: residual damping
            [~, ydotbar] = iCAT_task(eye(13), eye(13), Qp, ydotbar, zeros(13,1), 1e-4, 0.01, 10);

            % 5. Split velocities for vehicle and arm
            qdot = ydotbar(1:7);
            v_nu = ydotbar(8:13); % projected on the vehicle frame
        end

        function setCurrentAction(obj, actionIndex)
            % Switch to a different action
            if actionIndex >= 1 && actionIndex <= length(obj.actions)
                
                if actionIndex ~= obj.currentAction
                    % Logic for transition initialization
                    obj.previousAction = obj.currentAction;
                    obj.currentAction = actionIndex;
                    obj.t = 0;      % Reset timer
                    obj.k_in = 0;   % Force start
                    obj.k_out = 1;
                end
                
            else
                error('Action index out of range');
            end
        end
    end
end