function [solution6RWristPartitioned] = InverseKinematicsWristPartitioned( robotObject , HTM)

aDH = robotObject.rtbSerialLinkObject.a;
bDH = robotObject.rtbSerialLinkObject.d;
alphaDH = robotObject.rtbSerialLinkObject.alpha;


a1=aDH(1);a2=aDH(2);a3=aDH(3);a4=aDH(4);a5=aDH(5);a6=aDH(6);
b1=bDH(1);b2=bDH(2);b3=bDH(3);b4=bDH(4);b5=bDH(5);b6=bDH(6);
alpha1 = alphaDH(1);alpha2 = alphaDH(2);alpha3 = alphaDH(3);
alpha4 = alphaDH(4);alpha5 = alphaDH(5);alpha6 = alphaDH(6);


%% Input Position and Orientation of End-effector
% Orientaion Matrix 'Q'
Q = HTM(1:3,1:3);


xe = HTM(1,4);
ye = HTM(2,4);
ze = HTM(3,4);

%% Calculation of Wrist position from above
wrist = [xe ye ze]' - Q*[a6 b6*sin(alpha6) b6*cos(alpha6) ]';
xc = wrist(1);
yc = wrist(2);
zc = wrist(3);

%% CONSTANTS Defined on pg 189 S. K Saha Second Edition

A=2*a1*xc;
B=2*a1*yc;
C=2*a2*a3-2*b2*b4*sin(alpha2)*sin(alpha3);
D=2*a3*b2*sin(alpha2)+2*a2*b4*sin(alpha3);
E=a2^2+a3^2+b2^2+b3^2+b4^2-a1^2-xc^2-yc^2-(zc-b1)^2+2*b2*b3*cos(alpha2)+2*b2*b4*cos(alpha2)*cos(alpha3)+2*b3*b4*cos(alpha3);
F=yc*sin(alpha1);
G=-xc*sin(alpha1);
H=-b4*sin(alpha2)*sin(alpha3);
I=a3*sin(alpha2);
J=b2+b3*cos(alpha2)+b4*cos(alpha2)*cos(alpha3)-(zc-b1)*cos(alpha1);          %(6.46g)  p.188 book Saha Sir


K=4*a1^2*H^2+(sin(alpha1))^2*C^2;
L=4*a1^2*I^2+(sin(alpha1))^2*D^2;
M=2*(4*a1^2*H*I+(sin(alpha1))^2*C*D);
N=2*(4*a1^2*J*I+(sin(alpha1))^2*E*D);
P=2*(4*a1^2*J*I+(sin(alpha1))^2*E*D);
QQ=4*a1^2*J^2+(sin(alpha1))^2*E^2-4*a1^2*(sin(alpha1))^2*(xc^2+yc^2);  % Since Q used for Orientation

R=4*a1^2*(J-H)^2+(sin(alpha1))^2*(E-C)^2-4*(xc^2+yc^2)*a1^2*(sin(alpha1))^2;
S=4*(4*a1^2*I*(J-H)+(sin(alpha1))^2*D*(E-C));
T=2*(4*a1^2*(J^2-H^2+2*I^2)+(sin(alpha1))^2*(E^2-C^2+2*D^2)-4*(xc^2+yc^2)*a1^2*(sin(alpha1))^2);
U=4*(4*a1^2*I*(J+H)+(sin(alpha1))^2*D*(E+C));
V=4*a1^2*(J+H)^2+(sin(alpha1))^2*(E+C)^2-4*(xc^2+yc^2)*a1^2*(sin(alpha1))^2;

%% Quantic Equation for getting four solution of theta3
syms zeta
Z = eval(solve(R*zeta^4+S*zeta^3+T*zeta^2+U*zeta+V,zeta)); % (6.48a)
a = imag(Z);

temp = 1;

for i = 1:4
    if a(i)==0
        theta3=2*atan(Z(i));        
        THETA3(temp)=theta3;
        
        del1=-2*a1*sin(alpha1)*(xc^2+yc^2);
        A11=a2+a3*cos(theta3)+b4*sin(alpha3)*sin(theta3);
        A12=-a3*cos(alpha2)*sin(theta3)+b3*sin(alpha2)+b4*cos(alpha2)*sin(alpha3)*cos(theta3)+b4*sin(alpha2)*cos(alpha3);
        del2=A11^2+A12^2;
        
        if abs(del1)>0.0001
            c1=(1/del1)*(-G*(C*cos(theta3)+D*sin(theta3)+E)+ B*(H*cos(theta3)+I*sin(theta3)+J));
            s1=(1/del1)*(F*(C*cos(theta3)+D*sin(theta3)+E)- A*( H*cos(theta3)+I*sin(theta3)+J));
            theta1 = atan2(s1,c1);        
            THETA1(temp) = theta1;
            
            if abs(del2)>0.0001
                c2=(1/del2)*(A11*(xc*cos(theta1)+yc*sin(theta1)-a1)-A12*(-xc*cos(alpha1)*sin(theta1)+yc*cos(alpha1)*cos(theta1)+(zc-b1)*sin(alpha1)));
                s2=(1/del2)*(A12*(xc*cos(theta1)+yc*sin(theta1)-a1)+A11*(-xc*cos(alpha1)*sin(theta1)+yc*cos(alpha1)*cos(theta1)+(zc-b1)*sin(alpha1)));
                theta2 = atan2(s2,c2);                
                THETA2(temp) = theta2;
                
            end
            
        end
        
    end
    temp=temp+1;
end

Theta_123= [THETA1' THETA2' THETA3'] ;

%  Find the total number of valid solutions
numberOfPositionSolutions = size(Theta_123,1);
solution6RWristPartitioned = zeros(2*numberOfPositionSolutions,6);

for iters= 1:numberOfPositionSolutions
	WristToEEOrientation = ((rotz(Theta_123(iters,3))*rotx(alpha3))'*(rotz(Theta_123(iters,2))*rotx(alpha2))'*(rotz(Theta_123(iters,1))*rotx(alpha1))')*Q; 
	WristSolution = InverseKinematicsWrist(WristToEEOrientation);
	
	solution6RWristPartitioned(2*(iters-1)+1,1:3) = Theta_123(iters,1:3);
    solution6RWristPartitioned(2*(iters-1)+1,4) =  WristSolution(2,1);
	solution6RWristPartitioned(2*(iters-1)+1,5:6) = WristSolution(1,2:3);
	
	solution6RWristPartitioned(2*(iters-1)+2,1:3) = Theta_123(iters,1:3);
    solution6RWristPartitioned(2*(iters-1)+2,4) = WristSolution(1,1);
	solution6RWristPartitioned(2*(iters-1)+2,5:6) = WristSolution(2,2:3);
	
	
end

end