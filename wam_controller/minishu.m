clear all 
close all 


%% vehicle parameters
m=1.11;%kg

%kg.m^2
Ixx=1.6*10^-2;
Iyy=1.0*10^-2;
Izz=2.4*10^-2; 


b=.1;

%m
d=0.179;
l=0.075;
h=0.022;

g=9.81;%N/Kg

%% PID gains
KI_z=0;
KP_z=10;
KD_z=5;

KI_theta=10;
KP_theta=10;
KD_theta=10;

KI_phi=100;
KP_phi=10;
KD_phi=30;

KI_ro=10;
KP_ro=10;
KD_ro=10;


%% initialize simulation
Ts=1/500; 

Tf=5;

t_d=0:Ts:Tf-Ts ; 

z_d=0.3;

theta_d=pi/2;
phi_d=pi/4;
ro_d=0; 

n1= length(t_d) ;



x(1:n1)=0;
xd(1:n1)=0;
xdd(1:n1)=0;

y(1:n1)=0;
yd(1:n1)=0;
ydd(1:n1)=0;

z(1:n1)=0;
zd(1:n1)=0;
zdd(1:n1)=0;

phi(1:n1)=0;
phid(1:n1)=0;
phidd(1:n1)=0;

theta(1:n1)=pi;
thetad(1:n1)=0;
thetadd(1:n1)=0;

ro(1:n1)=0;
rod(1:n1)=0;
rodd(1:n1)=0;


theta1(1:n1)=pi;
theta1d(1:n1)=pi;

theta2(1:n1)=pi;
theta2d(1:n1)=pi;

u=zeros(4,n1);
u1=u;

%% simulation
for tk=1:n1-1 
  if tk==1
       U2p=KP_phi*(phi_d-phi(tk))*Ixx;
       U3p=(KP_theta*(theta(tk)-theta_d) +KD_theta*thetad(tk))*Iyy +0*xd(tk) ;
        U4p=KP_ro*(ro_d-ro(tk))*Izz;
    
         U1p=-m*(-g + KP_z*(-z_d +z(tk)) + KD_z*(zd(tk))); 
  else 
   
  U2p=(KI_phi*(phi_d-phi(tk-1))+KP_phi*(phi_d-phi(tk)) -KD_phi*phid(tk))*Ixx;
  U3p=(KI_theta*(theta(tk-1)-theta_d)+KP_theta*(theta(tk)-theta_d) +KD_theta*thetad(tk))*Iyy/h +0*xd(tk) ;
  U4p=(KI_ro*(ro_d-ro(tk-1))+KP_ro*(ro_d-ro(tk)) -KD_ro*rod(tk))*Izz;
    
  U1p=-m*(-g + KP_z*(-z_d +z(tk))+KI_z*(-z_d+z(tk-1)) + KD_z*(zd(tk))); 
  
  end 
  
  u(:,tk)= [U1p;U2p;U3p;U4p];
  
  U1=U1p;
  U2=(U4p+b/d*U2p)/(b^2/d+d);
  U3=U3p;
  U4=(U4p-d/b*U2p)/(b+d^2/b);
  
  f1(:,tk)=sqrt((U1+U2)^2 +(U3+U4)^2)/2;
  f2(:,tk)=sqrt((U1-U2)^2 +(U3-U4)^2)/2;
  
  if f1(:,tk)>10 
      f1(:,tk)=10 ; 
  end 
  if f2(:,tk)>10 
      f2(:,tk)=10 ; 
  end 
  
  theta1_d(:,tk)=asin((U3+U4)/(2*f1(:,tk)));
  theta2_d(:,tk)=asin((U3-U4)/(2*f2(:,tk)));
  
  theta1p(:,tk)=acos((U1+U2)/(2*f1(:,tk)));
  theta2p(:,tk)=acos((U1-U2)/(2*f2(:,tk)));
  
  %do a reconstruction of control inputs. 
  
  U11= f1(:,tk)*cos(theta1_d(:,tk)) +f2(:,tk)*cos(theta2_d(:,tk));
  U21= f1(:,tk)*cos(theta1_d(:,tk)) -f2(:,tk)*cos(theta2_d(:,tk));
  U31= f1(:,tk)*sin(theta1_d(:,tk)) +f2(:,tk)*sin(theta2_d(:,tk));
  U41= f1(:,tk)*sin(theta1_d(:,tk)) -f2(:,tk)*sin(theta2_d(:,tk));
  
  U1p=U11;
  U2p=b*U21-d*U41;
  U3p=U31;
  U4p=d*U21+b*U41;
  
  u1(:,tk)= [U1p;U2p;U3p;U4p];
  
% reconstructed control simulation
  x(tk+1)=xd(tk)*Ts+ x(tk); 
  xd(tk+1)=xdd(tk)*Ts+ xd(tk); 
  
  y(tk+1)=yd(tk)*Ts+ y(tk); 
  yd(tk+1)=ydd(tk)*Ts+ yd(tk); 
  
  z(tk+1)=zd(tk)*Ts+ z(tk); 
  zd(tk+1)=zdd(tk)*Ts+ zd(tk); 
  
  phi(tk+1)=phid(tk)*Ts+ phi(tk); 
  phid(tk+1)=phidd(tk)*Ts+ phid(tk); 
  
  theta(tk+1)=thetad(tk)*Ts+ theta(tk); 
  thetad(tk+1)=thetadd(tk)*Ts+ thetad(tk); 
  
  ro(tk+1)=rod(tk)*Ts+ ro(tk); 
  rod(tk+1)=rodd(tk)*Ts+ rod(tk); 
  
  theta1(tk+1)=theta1d(tk)*Ts+ theta1(tk); 
  theta2(tk+1)=theta2d(tk)*Ts+ theta2(tk); 
  
  
  xdd(tk+1)=1/m*cos(ro(tk))*cos(theta(tk))*U1p + 1/m*(cos(ro(tk))*sin(theta(tk))*cos(phi(tk))+sin(ro(tk))*sin(phi(tk)))*U3p -0.02*xd(tk)  ;
  ydd(tk+1)=1/m*sin(ro(tk))*cos(theta(tk))*U1p + 1/m*(sin(ro(tk))*sin(theta(tk))*cos(phi(tk))-cos(ro(tk))*sin(phi(tk)))*U3p ; 
  zdd(tk+1)=1/m*sin(theta(tk))*U1p +1/m*cos(theta(tk))*cos(phi(tk))*U3p - g ;
  
  phidd(tk+1)=1/Ixx*U2p;
  thetadd(tk+1)=-h/Iyy*U3p; 
  rodd(tk+1)=1/Izz*U4p; 
  
  theta1d(:,tk+1)=10*(theta1_d(:,tk)-theta1(:,tk));
  theta2d(:,tk+1)=10*(theta2_d(:,tk)-theta2(:,tk));
    
end 


%% plots



subplot(3,2,1)

plot(t_d,theta,t_d,ones(1,n1)*theta_d)
xlabel('t')
ylabel('theta');

subplot(3,2,2)


plot(t_d,phi,t_d,ones(1,n1)*phi_d)
xlabel('t')
ylabel('phi');


subplot(3,2,3)

plot(t_d,ro,t_d,ones(1,n1)*ro_d)
xlabel('t')
ylabel('ro');


subplot(3,2,4)

plot(t_d,z,t_d,ones(1,n1)*z_d)
xlabel('t')
ylabel('z');


subplot(3,2,5)

plot(t_d,x)
xlabel('t')
ylabel('x');


subplot(3,2,6)

plot(t_d,y)
xlabel('t')
ylabel('y');


figure 

plot(t_d,u)
xlabel('t')
ylabel('u');


figure 

subplot(4,1,1)
plot(t_d(1:end-1),theta1_d*180/pi,t_d,theta1*180/pi)
xlabel('t')
ylabel('theta1_d')

subplot(4,1,2)
plot(t_d(1:end-1),theta2_d*180/pi,t_d,theta2*180/pi)
xlabel('t')
ylabel('theta2_d')

subplot(4,1,3)
plot(t_d(1:end-1),f1)
xlabel('t')
ylabel('f1')


subplot(4,1,4)
plot(t_d(1:end-1),f2)
xlabel('t')
ylabel('f2')


u-u1;



