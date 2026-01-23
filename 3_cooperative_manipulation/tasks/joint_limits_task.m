classdef joint_limits_task < Task   
    %Tool position control for a single arm
    properties

    end

    methods
        function obj=joint_limits_task(robot_ID,taskID)
            obj.ID=robot_ID;
            obj.task_name=taskID;
        end

        function updateReference(obj, robot)
            q = robot.q;
            qmin = robot.jlmin;
            qmax = robot.jlmax;
             
            margin = 0.2;   % rad
            k = 0.5;
            qdot_ref = zeros(7,1);

            for i = 1:7                
                if q(i) < qmin(i) + margin
                    fprintf("Joint limits min exceeded for joint %d with value=%f\n", i, q(i));
                    qdot_ref(i) =  k * (qmin(i) + margin - q(i));
                elseif q(i) > qmax(i) - margin
                    fprintf("Joint limits max exceeded for joint %d with value=%f\n", i, q(i));
                    qdot_ref(i) = -k * (q(i) - (qmax(i) - margin));
                end
            end
    
            obj.xdotbar = Saturate(qdot_ref, 0.5);   
        end

        function updateJacobian(obj,robot)
            obj.J = eye(7);
        end

        function updateActivation(obj, robot)
            q = robot.q;
            qmin = robot.jlmin;
            qmax = robot.jlmax;
        
            margin = 0.2;
        
            A_diag = zeros(7,1);
        
            for i = 1:7
                dist = min(q(i) - qmin(i), qmax(i) - q(i));
        
                % fully active at limit, inactive outside margin
                A_diag(i) = DecreasingBellShapedFunction(0, margin, 0, 1, dist);
            end
        
            obj.A = diag(A_diag);
                    
        end
    end
end