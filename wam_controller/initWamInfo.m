function [wamInfo] = initWamInfo()
    % load in global variables from workspace
    global arm_home arm_home_norest hand_home hand_on arm_ready arm_kinematic_parameters arm_dynamic_parameters;
    
    if(isempty(arm_home) || isempty(arm_home_norest) || isempty(hand_home) || isempty(hand_on) || isempty(arm_ready) || isempty(arm_kinematic_parameters) || isempty(arm_dynamic_parameters))
        error('One or more WAM variables not loaded in workspace');
    end
    
    % update parameter d if the hand is on
    % creating a local variable so we don't wipeout the workspace variable
    dh_d = arm_kinematic_parameters.arm_dh_d;
    dyn = arm_dynamic_parameters;
     if(hand_on)
         hand_mass = 1.3;           % Schunk Gripper
         dh_d(7) = .061 + .0953;    % Schunk Gripper
         dyn.m(7) = dyn.m(7) + 1.3; % Schunk Gripper
         com = [0 0 .07];           % Schunk Gripper  (In 7th frame)
        dyn.I(:,:,7) = [ .003167342,          0,         0;       % Approximate Inertia Tensor for Schunk Gripper
                                  0, .001128942,         0;
                                  0,          0, .00258007;];

%         hand_mass = 1.2;                 % BarrettHand
%         dh_d(7) = .061 + .095;           % BarrettHand
%         dyn.m(7) = dyn.m(7) + hand_mass; % BarrettHand
%         com = [0 0 .06];                 % BarrettHand (In 7th frame)

        % Wooden foot
%        hand_mass = .254;
%        dh_d(7) = .061+.095;
%        dyn.m(7) = dyn.m(7) + hand_mass;
%        com  = [0 0 .06];

        dyn.r(:,:,7) = ((com.*hand_mass) + ([ 0.00014836,0.00007252,-0.00335185].*0.07548270))/(hand_mass + 0.07548270); % Use this line for WAM with Barrett hand
    end
    
    % create wam info struct
    % Make a local temporary d variable to access the fields easier
    dynInfo = struct('m',dyn.m,'r',dyn.r,'I',dyn.I,'Jm',dyn.Jm,'B',dyn.B);
    armDH = struct('a', arm_kinematic_parameters.arm_dh_a, 'd', dh_d, 'alpha', arm_kinematic_parameters.arm_dh_alpha);
    armInfo = struct('home', arm_home, 'home_norest', arm_home_norest, 'ready', arm_ready, 'dh', armDH,'dyn',dynInfo);
    handInfo = struct('home', hand_home);
    
    wamInfo = struct('arm', armInfo, 'hand', handInfo);
end