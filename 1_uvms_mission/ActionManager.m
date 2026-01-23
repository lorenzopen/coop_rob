classdef ActionManager < handle
    properties
        actions = {}        % cell array of actions (each action = stack of tasks)
        actionsName = {}    % name of the actions - array
        unifiedList = {}    % array containing the unified list of all tasks
        
        currentAction = 1   % index of currently active action
        previousAction = 1  % index of previously active action
        
        timeSinceSwitch = 0;  % time since last action switch
        transitionTime = 2.0    % duration of transition [s]
    end

    methods
        function addAction(obj, taskStack, action_name)
            % taskStack: cell array of tasks that define an action
            obj.actions{end+1} = taskStack;
            obj.actionsName{end+1} = char(action_name);
        end
        
        function addUnifyingTaskList(obj, unifiedList)
             obj.unifiedList = unifiedList;
        end

        function [v_nu, qdot] = computeICAT(obj, robot, dt)
            % Update time
            obj.timeSinceSwitch = obj.timeSinceSwitch + dt;
            t = obj.timeSinceSwitch;
            T = obj.transitionTime;

            % Retrieve task stacks
            unifiedTasks = obj.unifiedList;
            currTasks = obj.actions{obj.currentAction};
            prevTasks = obj.actions{obj.previousAction};

            % Membership flags (handle comparison!)
            inCurrent = cellfun(@(t) any(cellfun(@(c) c == t, currTasks)), unifiedTasks);
            inPrevious = cellfun(@(t) any(cellfun(@(p) p == t, prevTasks)), unifiedTasks);

            % Update tasks and compute transition activations
            for i = 1:length(unifiedTasks)
                task = unifiedTasks{i};
        
                task.updateReference(robot);
                task.updateJacobian(robot);
                task.updateActivation(robot);
        
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
                task.A = a * task.A;
            end

            % Perform ICAT (task-priority inverse kinematics)
            ydotbar = zeros(13,1);
            Qp = eye(13);
            for i = 1:length(unifiedTasks)
                t = unifiedTasks{i};
                [Qp, ydotbar] = iCAT_task(t.A, t.J, ...
                                           Qp, ydotbar, t.xdotbar, ...
                                           1e-4, 0.01, 10);
            end

            % Last task: residual damping
            [~, ydotbar] = iCAT_task(eye(13), eye(13), Qp, ydotbar, zeros(13,1), 1e-4, 0.01, 10);

            % Split velocities for vehicle and arm
            qdot = ydotbar(1:7);
            v_nu = ydotbar(8:13); % projected on the vehicle frame
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