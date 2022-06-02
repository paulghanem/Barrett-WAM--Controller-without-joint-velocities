function tau = dynamicsComp(hand_on,q,qd,qdd)
%-------------------------------------------------------------------------------------
%   SETUP THE ARM
%-------------------------------------------------------------------------------------    
    DH_a = [0 0 0.045 -0.045 0 0 0];
    DH_d = [0 0 .55 0 0.3 0 0.061];
    DH_alpha = [-pi/2 pi/2 -pi/2 pi/2 -pi/2 pi/2 0];

    m = [8.3936 4.8487 1.7251 2.17266212 0.35655692 0.40915886 0.07548270];
    
    com = zeros(3,7);
    com(:,1) = [0.3506e-3, 132.6795e-3, 0.6286e-3];
    com(:,2) = [-0.2230e-3, -21.3924e-3, 13.3754e-3];
    com(:,3) = [-38.7565e-3, 217.9078e-3, 0.0252e-3];
    com(:,4) = [0.00553408,0.00006822,0.11927695];
    com(:,5) = [0.00005483,0.02886286,0.00148493];
    com(:,6) = [-0.00005923,-0.01686123,0.02419052];
    com(:,7) = [ 0.00014836,0.00007252,-0.00335185];
    
    % Need Inertia matricies
    I = zeros(3,3,7);
    I(:,:,1) = [ 95157.4294e-6,   246.1404e-6,   -95.0183e-6 ;
                   246.1404e-6, 92032.3524e-6,  -962.6725e-6 ;
                   -95.0183e-6,  -962.6725e-6, 59290.5997e-6 ];
    I(:,:,2) = [ 29326.8098e-6,   -43.3994e-6,  -129.2942e-6 ;
                   -43.3994e-6, 20781.5826e-6,  1348.6924e-6 ;
                  -129.2942e-6,  1348.6924e-6, 22807.3271e-6 ];
    I(:,:,3) = [ 56662.2970e-6, -2321.6892e-6,     8.2125e-6 ;
                 -2321.6892e-6,  3158.0509e-6,   -16.6307e-6 ;
                     8.2125e-6,   -16.6307e-6, 56806.6024e-6 ];
    I(:,:,4) = [  0.01067491,  0.00004503, -0.00135557 ;
                  0.00004503,  0.01058659, -0.00011002 ;
                 -0.00135557, -0.00011002,  0.00282036 ];
    I(:,:,5) = [  0.00037112, -0.00000008, -0.00000003 ;
                 -0.00000008,  0.00019434, -0.00001613 ;
                 -0.00000003, -0.00001613,  0.00038209 ];
    I(:,:,6) =  [  0.00054889,  0.00000019, -0.00000010 ;
                   0.00000019,  0.00023846, -0.00004430 ;
                  -0.00000010, -0.00004430,  0.00045133 ];
    I(:,:,7) = [ 0.00003911, 0.00000019, 0.00000000 ;      % WAM without hand
                 0.00000019, 0.00003877, 0.00000000 ;
                 0.00000000, 0.00000000, 0.00007614 ];
             
    %Link Motor Inertia
    Jm = zeros(7,1);
    Jm(1) = 0.00011631;
    Jm(2) = 0.00011831;
    Jm(3) = 0.00011831;
    Jm(4) = 0.00010686;
    Jm(5) = 0.00001685;
    Jm(6) = 0.00001745;
    Jm(7) = 0.00000142;
    
    %Link Gear Ratio
    % The differential drive mechanism was ignored here because it's not
    % supported by the robotic toolbox yet.
    % Please refer to: http://www.me.unm.edu/~starr/research/WAM_MassParams_AA-00.pdf
    G = zeros(7,1);
    G(1) =  42.00;
    G(2) =  28.25;
    G(3) =  28.25;
    G(4) =  18.00;
    G(5) =  9.70;
    G(6) =  9.70;
    G(7) =  14.93;  % WAM without hand
    
    if hand_on
        hand_mass = 1.2;
        DH_d(7) = DH_d(7) + .108; % Adding the distance to the hand frame
        com(:,7) = ([0;0;-0.04].*hand_mass + com(:,7).*m(7))/(hand_mass + m(7)); % Finding the combined center of mass
        m(7) = m(7) + hand_mass; % Adding the mass of the hand
        I(:,:,7) = [ .003167342,          0,         0;       % Approximate Inertia Tensor for Schunk Gripper
                              0, .001128942,         0;
                              0,          0, .00258007;];
    end
    
%-------------------------------------------------------------------------------------
%  END SETUP OF THE ARM
%-------------------------------------------------------------------------------------

    z0 = [0;0;1];
    grav = [0 0 9.81];
	fext = zeros(6, 1);
    n = 7;
    
    tau = zeros(n,1);
    
    Fm = zeros(3,7);
    Nm = zeros(3,7);
    pstarm = zeros(3,7);
    Rm = zeros(3,3,7);
    w = zeros(3,1);
    wd = zeros(3,1);
    v = zeros(3,1);
    vd = grav(:);
    
    
    %
	% init some variables, compute the link rotation matrices
	%
    for j=1:n,
        a = DH_a(j);
        d = DH_d(j);
        alpha = DH_alpha(j);
        Tj = getTransformDH(a, d, alpha, q(j));
        
        pstar = [a; d*sin(alpha); d*cos(alpha)];
        if j == 1,
            pstar = eye(3) * pstar;
            Tj = eye(4) * Tj;
        end
        pstarm(:,j) = pstar;
        Rm(:,:,j) = Tj(1:3,1:3);
    end
    
    %
	%  the forward recursion
	%
    for j=1:n,
        Rt = Rm(:,:,j)';	% transpose!!
        pstar = pstarm(:,j);
        r = com(:,j)';

        %
        % statement order is important here
        %
        % revolute axis
        wd = Rt*(wd + z0*qdd(j) + cross(w,z0*qd(j)));
        w = Rt*(w + z0*qd(j));
        %v = cross(w,pstar) + Rt*v;
        vd = cross(wd,pstar) + cross(w, cross(w,pstar)) + Rt*vd;

        %whos
        vhat = cross(wd,r') + cross(w,cross(w,r')) + vd;
        F = m(j)*vhat;

        N = I(:,:,j)*wd + cross(w,I(:,:,j)*w);
        Fm(:,j) = F;
        Nm(:,j) = N;
%         if debug
%             fprintf('w: '); fprintf('%.3f ', w)
%             fprintf('\nwd: '); fprintf('%.3f ', wd)
%             fprintf('\nvd: '); fprintf('%.3f ', vd)
%             fprintf('\nvdbar: '); fprintf('%.3f ', vhat)
%             fprintf('\n');
%         end
    end
    
    
    %
	%  the backward recursion
	%

    fext = fext(:);
    f = fext(1:3);		% force/moments on end of arm
    nn = fext(4:6);

    for j=n:-1:1,
        pstar = pstarm(:,j);

        %
        % order of these statements is important, since both
        % nn and f are functions of previous f.
        %
        if j == n,
            R = eye(3,3);
        else
            R = Rm(:,:,j+1);
        end
        r = com(:,j)';
        nn = R*(nn + cross(R'*pstar,f)) + cross(pstar+r',Fm(:,j)) + Nm(:,j);
        f = R*f + Fm(:,j);

        R = Rm(:,:,j);
        % revolute
        tau(p,j) = nn'*(R'*z0) + G^2 * Jm(j)*qdd(j);  %NOFRICTION% + abs(G) * friction(link, qd(j));
    end

end