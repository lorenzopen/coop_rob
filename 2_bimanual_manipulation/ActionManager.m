classdef ActionManager < handle
    properties
        actions = {}       % cell array of actions (each action = stack of tasks)
        actionsName = {}   % names of the actions
        unifiedList = {}   % Global list defining priority order
        
        currentAction = 1  % index of currently active action
        previousAction = 1 
        
        t = 0              % Transition timer
        T_trans = 2.0      % Transition duration [s]
        k_in = 1           % Activation scalar for incoming action
        k_out = 0          % Activation scalar for outgoing action
        
        isBinaryTransition = false % flag for binary (no-smooth) transitions
    end

    methods
        function obj = ActionManager()
        end

        function addAction(obj, taskStack, action_name)
            obj.actions{end+1} = taskStack;
            obj.actionsName{end+1} = char(action_name);
        end

        function addUnifiedList(obj, list)
            obj.unifiedList = list;
        end
        
        function setBinaryTransition(obj, isBinary)
            obj.isBinaryTransition = isBinary;
        end

        function [ydotbar] = computeICAT(obj, bm_system, dt)
            % 1. Update timer
            obj.t = obj.t + dt;

            % 2. Update references, Jacobians, activations for ALL tasks
            for i = 1:length(obj.unifiedList)
                obj.unifiedList{i}.updateReference(bm_system);
                obj.unifiedList{i}.updateJacobian(bm_system);
                obj.unifiedList{i}.updateActivation(bm_system);
            end

            % 3. Compute transition scalars
            if obj.currentAction ~= obj.previousAction && obj.t <= obj.T_trans
                if obj.isBinaryTransition
                    % Binary: switch immediately at the start of transition
                    obj.k_in = 1;
                    obj.k_out = 0;
                else
                    % Smooth: Bell-shaped functions
                    obj.k_in = IncreasingBellShapedFunction(0, obj.T_trans, 0, 1, obj.t);
                    obj.k_out = DecreasingBellShapedFunction(0, obj.T_trans, 0, 1, obj.t);
                end
            else
                % Steady state
                obj.k_in = 1; 
                obj.k_out = 0;
            end

            % Retrieve stacks
            currTasks = obj.actions{obj.currentAction};
            prevTasks = obj.actions{obj.previousAction};

            % 4. Perform ICAT (task-priority inverse kinematics)
            % Note: dimension updated to 14 for the new exercise
            ydotbar = zeros(14,1);
            Qp = eye(14);
            
            for i = 1:length(obj.unifiedList)
                task = obj.unifiedList{i};

                % Check membership using handle comparison
                inCurr = any(cellfun(@(x) x == task, currTasks));
                inPrev = any(cellfun(@(x) x == task, prevTasks));
                
                % Determine activation weight w
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

            % 5. Last task: residual damping
            [~, ydotbar] = iCAT_task(eye(14), eye(14), Qp, ydotbar, zeros(14,1), 1e-4, 0.01, 10);
        end

        function setCurrentAction(obj, action_name)
            % Switch using name lookup
            idx = find(strcmp(obj.actionsName, action_name), 1);

            if isempty(idx)
                error('Action "%s" not found.', action_name);
            end

            if idx ~= obj.currentAction
                obj.previousAction = obj.currentAction;
                obj.currentAction = idx;
                obj.t = 0;      % Reset timer
                % Initialize transition states
                if obj.isBinaryTransition
                    obj.k_in = 1; obj.k_out = 0;
                else
                    obj.k_in = 0; obj.k_out = 1;
                end
            end
        end
    end
end