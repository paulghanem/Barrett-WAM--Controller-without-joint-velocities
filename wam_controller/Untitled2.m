function G1=G1(q)
DH_a = [0 0 0.045 -0.045 0 0 0];
    DH_d = [0 0 .55 0 0.3 0 0.061];
    DH_alpha = [-pi/2 pi/2 -pi/2 pi/2 -pi/2 pi/2 0];

    m = [10.7676 3.87 1.8022 2.4 0.1237 0.4979886 0.07548270];
    
    com = zeros(3,7);
    com(:,1) = [-0.00443422; 0.12189039; -0.00066489];
    com(:,2) = [-0.00236983; 0.03105614; 0.01542114 ];
    com(:,3) = [-0.03825858; 0.20750770; 0.00003309];
    com(:,4) = [ 0.00498512;-0.00022942;0.13271662 ];
    com(:,5) = [0.00008921;0.00511217;0.00435824];
    com(:,6) = [-0.00012262;-0.01703194;0.02468336];
    com(:,7) = [-0.00007974;0.00016313;-0.00323552 ];
     
      hand_mass = 1.385;
        DH_d(7) = DH_d(7) + .0953; % Adding the distance to the hand frame for the Schunk Gripper
        com(:,7) = (com(:,7).*hand_mass + com(:,7).*m(7))/(hand_mass + m(7)); % Finding the combined center of mass
        m(7) = m(7) + hand_mass;  

C1 = 9.81*(m(4)*com(3,4) + m(5)*DH_d(5) + m(5)*com(3,5));
C2 = 9.81*(m(4)*DH_a(4) + m(4)*com(1,4) + m(5)*DH_a(4) + m(5)*com(1,5));
C3 = 9.81*(m(3)*com(3,3) + m(4)*com(2,4) + m(5)*com(2,5));
C4 = 9.81*(m(3)*DH_a(3) + m(3)*com(1,3) + m(4)*DH_a(3) + m(5)*DH_a(3));
C5 = 9.81*(m(2)*com(3,2) + m(3)*DH_d(3) - m(3)*com(2,3) + m(4)*DH_d(3) + m(5)*DH_d(3));
C6 = 9.81*(m(2)*com(1,2));

lambda1=-cos(q(4))*sin(q(2))-cos(q(2))*cos(q(3))*sin(q(4));
lambda2=sin(q(2))*sin(q(4))-cos(q(2))*cos(q(3))*cos(q(4));
lambda3=cos(q(2))*sin(q(3));
lambda4=-cos(q(2))*cos(q(3));
lambda5=-sin(q(2));
lambda6=-cos(q(2));
beta1=sin(q(2))*sin(q(3))*sin(q(4));
beta2=cos(q(4))*sin(q(2))*sin(q(3));
beta3=cos(q(3))*sin(q(2));
beta4=sin(q(2))*sin(q(3));
gamma1=-cos(q(3))*cos(q(4))*sin(q(2))-cos(q(2))*sin(q(4));
gamma2=cos(q(3))*sin(q(2))*sin(q(4))-cos(q(2))*cos(q(4));
