function [Ha, Hb, C] = computeRigidGraspConstraints(armA, armB)
    % Compute rigid grasp constraints for two arms
    % Ha: constraint matrix for armA
    % Hb: constraint matrix for armB
    % C: combined constraint matrix

    % Get Jacobians for both arms
    Jt_A = armA.wJo;
    Jt_B = armB.wJo;

    % Compute the constraint matrices
    Ha = Jt_A*pinv(Jt_A);
    Hb = Jt_B*pinv(Jt_B);
    C = [Ha -Hb];
end