classdef panda_arm < handle
    % Franka Emika Panda Kinematic Model

    properties
        %% Rigid Body Tree
        robot_model
        %% --- State variables ---
        q
        qdot
        xdot
        xdot_coop

        v_coop
        wTo_partner
        %% --- Geometry ---
        wTb
        %% --- Limits ---
        jlmin
        jlmax
        %% ---Transformations ---
        bTe
        wTe
        eTt
        wTt
        %% ---Goals---
        wTg
        wTog
        wTo
        robot_type
        tTo
        wJe
        wJt
        wJo
        %% Sensor variables
        alt
        dist_to_goal
        rot_to_goal
    end

    methods
        function obj = panda_arm(model,wTb)
            % Constructor
            obj.robot_model = model;
            obj.wTb = wTb;
            obj.wJe = zeros(6,7);
            obj.wJt = zeros(6,7); 
            obj.wJo = zeros(6,7);
            
            %Initialize Default State
            obj.q=[0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';
            obj.qdot=[0 0 0 0 0 0 0]';
            
            %Get current transformation from world frame to end effector
            %Frame
            obj.bTe=getTransform(obj.robot_model.franka,[obj.q',0,0],'panda_link7');
            obj.wTe=obj.wTb*obj.bTe;

            %Default Limits
            obj.jlmin=[-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
            obj.jlmax=[2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];

            % FIXED END EFFECTOR
            theta = deg2rad(-44.9949);  % FIXED ANGLE BETWEEN EE AND TOOL 
            tool_length = 0.2104;       % FIXED DISTANCE BETWEEN EE AND TOOL

            % TO DO: Define trasnformation matrix from ee to tool, and
            % transformation from world frame to tool
            obj.eTt = [cos(theta) -sin(theta) 0     0;
                       sin(theta) cos(theta)  0     0;
                       0            0         1     tool_length;
                       0            0         0     1];
            obj.wTt = obj.wTe * obj.eTt;

            obj.xdot_coop = zeros(6,1);

            obj.tTo = [];
          
        end

        function setGoal(obj,obj_position,obj_orientation,arm_dist_offset,arm_rot_offset)
            % Set goal positions and orientations for arm 
            obj.wTo=[[obj_orientation obj_position]; 0 0 0 1];
            obj.wTg=obj.wTo*[[arm_rot_offset arm_dist_offset]; 0 0 0 1];
        end
        
        function set_obj_goal(obj,wTog)
            % Set goal positions and orientations for the object
            obj.wTog = wTog;
        end


        function update_transform(obj)
            % Compute forward kinematics of the robot

            obj.bTe=getTransform(obj.robot_model.franka,[obj.q',0,0],'panda_link7');
            obj.wTe=obj.wTb*obj.bTe;
            obj.wTt =obj.wTe*obj.eTt; 
            if(~isempty(obj.tTo))
                obj.wTo = obj.wTt * obj.tTo;
            end       
        end
        
        function update_jacobian(obj)
            % Compute Differential kinematics from the base frame to the
            % Tool Frame
            bJe = geometricJacobian(obj.robot_model.franka,[obj.q',0,0],'panda_link7');%DO NOT EDIT
            % 2. Ruotiamo lo Jacobiano nel World Frame
            wRb = obj.wTb(1:3,1:3);
            Rot_block = [wRb zeros(3); zeros(3) wRb];
            obj.wJe = Rot_block * bJe(:, 1:7);
            
            % 3. Trasporto dall'EE al Tool (wJt)
            % Vettore r_te: da EE a Tool espresso in World Frame
            w_r_te = obj.wTe(1:3,1:3) * obj.eTt(1:3,4);
            
            % Matrice trasporto 
            Ste = [eye(3) zeros(3); -skew(w_r_te) eye(3)];
            obj.wJt = Ste * obj.wJe;
            if ~isempty(obj.tTo)
                % Vettore r_ot: da Tool a Oggetto espresso in World Frame
                % tTo(1:3,4) è il vettore traslazione nel frame tool localmente
                % Lo ruotiamo con wTt per averlo nel world
                w_r_ot = obj.wTt(1:3,1:3) * obj.tTo(1:3,4);
                
                Sot = [eye(3) zeros(3); -skew(w_r_ot) eye(3)];
                obj.wJo = Sot * obj.wJt;
            else
                % Se non abbiamo l'oggetto, wJo è uguale al tool
                obj.wJo = obj.wJt; 
            end
        
        end
                
        function compute_object_frame(obj)
            % Define the object frame as a rigid body attached to the tool frame
            obj.tTo = obj.wTt \ obj.wTo;  % tTo = wTt^-1 * wTo
        end    
        
        function [xdotbar] = compute_desired_refVelocity(obj)
            % Compute desired object velocity for cooperative manipulation
            [v_ang, v_lin] = CartError(obj.wTog, obj.wTo); 

            % Desired object velocity
            xdotbar = 0.5 * [v_ang; v_lin]; % Gain aumentato per reattività

            % Saturation
            xdotbar(1:3) = Saturate(xdotbar(1:3), 0.3);
            xdotbar(4:6) = Saturate(xdotbar(4:6), 0.3);
        end
    end
end