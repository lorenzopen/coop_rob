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

        function addUnifyingTaskList(obj, list)
            % Nota: ho rinominato il metodo per coerenza col main (era addUnifiedList)
            obj.unifiedList = list;
        end
        
        function setBinaryTransition(obj, isBinary)
            obj.isBinaryTransition = isBinary;
        end

        function [ydotbar] = computeICAT(obj, robot, dt)
            % MODIFICA ES 3: 'robot' ora è il singolo braccio (7 DOF)
            
            % 1. Determina la dimensione del problema dinamicamente
            n = length(robot.q); 

            % 2. Update timer
            obj.t = obj.t + dt;

            % 3. Update references, Jacobians, activations for ALL tasks
            for i = 1:length(obj.unifiedList)
                % Chiama i metodi dei task passando il singolo robot
                obj.unifiedList{i}.updateReference(robot);
                obj.unifiedList{i}.updateJacobian(robot);
                obj.unifiedList{i}.updateActivation(robot);
            end

            % 4. Compute transition scalars
            if obj.currentAction ~= obj.previousAction && obj.t <= obj.T_trans
                if obj.isBinaryTransition
                    obj.k_in = 1;
                    obj.k_out = 0;
                else
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

            % 5. Perform ICAT (task-priority inverse kinematics)
            % MODIFICA ES 3: Usa dimensione n (7) invece di 14 fisso
            ydotbar = zeros(n,1);
            Qp = eye(n);
            
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

                if w == 0 && inCurr
                    fprintf('ATTENZIONE: Il task %s dovrebbe essere attivo ma w=0!\n', task.task_name);
                end

                
                % Execute iCAT only if task is active
                if w > 1e-6
                    % iCAT_task gestisce le dimensioni basandosi sulle matrici passate
                    [Qp, ydotbar] = iCAT_task(task.A * w, task.J, Qp, ydotbar, task.xdotbar, 1e-4, 0.01, 10);
                end
            end

            % 6. Last task: residual damping
            % MODIFICA ES 3: eye(n) invece di eye(14)
            [~, ydotbar] = iCAT_task(eye(n), eye(n), Qp, ydotbar, zeros(n,1), 1e-4, 0.01, 10);
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
                
                if obj.isBinaryTransition
                    obj.k_in = 1; obj.k_out = 0;
                else
                    obj.k_in = 0; obj.k_out = 1;
                end
            end
        end
        
        % Metodo helper utile per debug
        function task = getTaskByName(obj, name_string)
            task = [];
            for i = 1:length(obj.unifiedList)
                if strcmp(obj.unifiedList{i}.task_name, name_string)
                    task = obj.unifiedList{i};
                    return;
                end
            end
        end
    end
end