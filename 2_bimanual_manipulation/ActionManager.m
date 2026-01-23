classdef ActionManager < handle
    properties
        actions = {}        % cell array of actions (each action = stack of tasks)
        actionsName = {}    % name of the actions - array
        unifiedList = {}    % array containing the unified list of all tasks
        
        currentAction = 1   % index of currently active action
        previousAction = 1  % index of previously active action
        
        timeSinceSwitch = 0;  % time since last action switch
        transitionTime = 2.0    % duration of transition [s]
        isBinaryTransition = false  % flag for binary (no-smooth) transitions
    end

    methods

        function obj = ActionManager()
        end

        function addAction(obj, taskStack, action_name)
            % taskStack: cell array of tasks that define an action
            obj.actions{end+1} = taskStack;
            obj.actionsName{end+1} = char(action_name);
        end

        function addUnifyingTaskList(obj, unifiedList)
             obj.unifiedList = unifiedList;
        end
        
        function setBinaryTransition(obj, isBinary)
            % Set binary transition mode for the next action switch
            % Binary: no smooth transition (activation 0 or 1)
            % Smooth: Bell-shaped function transition over transitionTime
            obj.isBinaryTransition = isBinary;
        end

        function [ydotbar] = computeICAT(obj, bm_system, dt)
             % Update time
            obj.timeSinceSwitch = obj.timeSinceSwitch + dt;
            t = obj.timeSinceSwitch;
            T = obj.transitionTime;
            
            % Retrieve task stacks
            unifiedTasks = obj.unifiedList;
            currTasks = obj.actions{obj.currentAction};
            prevTasks = obj.actions{obj.previousAction};
            
            % Membership flags (handle comparison)
            inCurrent = cellfun(@(t) any(cellfun(@(c) c == t, currTasks)), unifiedTasks);
            inPrevious = cellfun(@(t) any(cellfun(@(p) p == t, prevTasks)), unifiedTasks);
            tasks = obj.actions{obj.currentAction};

            % Update tasks and compute transition activations
            for i = 1:length(unifiedTasks)
                task = unifiedTasks{i};
        
                task.updateReference(bm_system);
                task.updateJacobian(bm_system);
                task.updateActivation(bm_system);
        
                if obj.isBinaryTransition
                    % Binary transition: immediate 0 to 1 switch
                    if inCurrent(i) && ~inPrevious(i)
                        % Appearing task: immediately active
                        a = 1;
                    elseif ~inCurrent(i) && inPrevious(i)
                        % Disappearing task: immediately inactive
                        a = 0;
                    elseif inCurrent(i) && inPrevious(i)
                        % Task in both actions: fully active
                        a = 1;
                    else
                        % Task not present in either: inactive
                        a = 0;
                    end
                else
                    % Smooth transition: Bell-shaped function
                    if inCurrent(i) && ~inPrevious(i)
                        % Appearing task
                        a = IncreasingBellShapedFunction(0, T, 0, 1, t);
                    elseif ~inCurrent(i) && inPrevious(i)
                        % Disappearing task
                        a = DecreasingBellShapedFunction(0, T, 0, 1, t);    
                    elseif inCurrent(i) && inPrevious(i)
                        % Task in both actions: fully active
                        a = 1;
                    else
                        % Task not present in either: inactive
                        a = 0;
                    end
                end
                task.A = a * task.A;
            end
            
            % 2. Perform ICAT (task-priority inverse kinematics) for the current Action
            ydotbar = zeros(14,1);
            Qp = eye(14);
            for i = 1:length(tasks)
                 % TRANSITION FROM PREVIOUS ACTION SET TO NEXT ACTION SET
                [Qp, ydotbar] = iCAT_task(tasks{i}.A, tasks{i}.J, ...
                                           Qp, ydotbar, tasks{i}.xdotbar, ...
                                           1e-4, 0.01, 10);
            end
            % 3. Last task: residual damping
            [~, ydotbar] = iCAT_task(eye(14), eye(14), Qp, ydotbar, zeros(14,1), 1e-4, 0.01, 10);
        end

        function setCurrentAction(obj, action_name)           
            idx = find(strcmp(obj.actionsName, action_name), 1);

            if isempty(idx)
                error('Action "%s" not found.', action_name);
            end

            if idx ~= obj.currentAction
                obj.previousAction = obj.currentAction;
                obj.currentAction  = idx;
                obj.timeSinceSwitch = 0;   % reset transition timer
            end
        end
    end
end
