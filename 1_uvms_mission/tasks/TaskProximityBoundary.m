classdef TaskProximityBoundary < Task
    properties

    end

    methods
        function updateReference(obj, robot)
            [~,v_lin] = CartError(robot.wTg, robot.wTv);

            obj.xdotbar = - 0.4 * v_lin(1:2);
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        function updateJacobian(obj, robot)
            Jt_a  = zeros(2,7);
            J_v = [-robot.wTv(1:2, 1:3) zeros(2,3)];
            obj.J = [Jt_a J_v];
        end

        function updateActivation(obj, robot)
            [~,v_lin] = CartError(robot.wTg, robot.wTv);
            dist = norm(v_lin(1:2));
            obj.A = eye(2) * IncreasingBellShapedFunction(1.3,1.7,0,1,dist);
        end
    end
end

