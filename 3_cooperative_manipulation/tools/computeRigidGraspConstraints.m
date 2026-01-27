function [Ha, Hb, C] = computeRigidGraspConstraints(armA, armB)
    % computeRigidGraspConstraints
    % Calcola le matrici per il coordinamento nel FRAME DELL'OGGETTO.
    %
    % Dato che stiamo coordinando le velocità dell'oggetto (v_obj_L e v_obj_R),
    % il vincolo è semplicemente che devono essere identiche:
    % v_obj_L - v_obj_R = 0
    
    % Ha e Hb sono matrici di selezione/proiezione per l'output.
    % Poiché vogliamo che l'uscita (v_coop) sia ancora una velocità dell'oggetto,
    % queste devono essere matrici Identità.
    Ha = eye(6);
    Hb = eye(6);

    % Matrice di Vincolo C:
    % C * [v_obj_L; v_obj_R] = 0
    % [I  -I] * [v_L; v_R] = v_L - v_R = 0
    C = [eye(6), -eye(6)];
end