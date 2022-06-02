clc
clear all 

Ts=0.002;
t_d=0:Ts:6;
t_d1=0:Ts:7;
fs=500;
n = length(t_d);
n1=length(t_d1);
 x1 = [[-0.11060;-2;-0.19200;3.1;0.28250;0.09690;-0.27980]*ones(1,n-1)];
 x2=zeros(7,n);

x= [-0.1106+1+sin((pi/4)*t_d-pi/2);
    -0.932+sin((pi/4)*t_d-pi/2);
    -0.1920+0.5+0.5*sin((pi/4)*t_d-pi/2);
   2.1+sin((pi/4)*t_d+pi/2);
   0.2825*ones(1,n);
   0.0969*ones(1,n);
   -0.2798*ones(1,n)];
    
    
xd= [(pi/4)*cos((pi/4)*t_d-pi/2);
    (pi/4)*cos((pi/4)*t_d-pi/2);
    0.5*(pi/4)*cos((pi/4)*t_d-pi/2);
    (pi/4)*cos((pi/4)*t_d+pi/2);
    zeros(3,n)];


xdd=[-((pi/4).^2)*sin((pi/4)*t_d-pi/2);
     -((pi/4).^2)*sin((pi/4)*t_d-pi/2);
     -0.5*((pi/4).^2)*sin((pi/4)*t_d-pi/2);
     -((pi/4).^2)*sin((pi/4)*t_d+pi/2);
     zeros(3,n)];

K1=diag([30,30,30,30,30,30,30]);
K2=K1;


 for tk = 1:n
     
  

u(:,tk)= transpose(G(x1(:,tk)))+ D(x1(:,tk))*(xdd(:,tk)-(K1+K2)*(x2(:,tk)-xd(:,tk))-(eye(7)+K1*K2)*(x1(:,tk)-x(:,tk)));

x1(:,tk+1) =  x2(:,tk)*Ts + x1(:,tk);
            
 x2(:,tk+1) = (-inv(D(x1(:,tk)))*(transpose(G(x1(:,tk))))+ inv(D(x1(:,tk))+diag([0.20519,0.094428,0.094428,0.03,0.001,0.001,0.0003]))*u(:,tk))*Ts + x2(:,tk);
 end 
 
 plot(t_d,x(1,1:n),t_d,x1(1,1:n))
 xlabel('time')
ylabel('position')

legend('desired position','actual position')

title('joint1 simulation ')



figure
 plot(t_d,x(2,1:n),t_d,x1(2,1:n))
 xlabel('time')
ylabel('position')

legend('desired position','actual position')

title('joint2 simulation ')

figure
 plot(t_d,x(3,1:n),t_d,x1(3,1:n))
 xlabel('time')
ylabel('position')

legend('desired position','actual position')

title('joint3 simulation ')
 
figure
 plot(t_d,x(4,1:n),t_d,x1(4,1:n))
 xlabel('time')
ylabel('position')

legend('desired position','actual position')

title('joint4 simulation ')
 


figure
 plot(t_d,x(5,1:n),t_d,x1(5,1:n))
 xlabel('time')
ylabel('position')

legend('desired position','actual position')

title('joint5 simulation ')
 

 figure
 plot(t_d,x(6,1:n),t_d,x1(6,1:n))
 xlabel('time')
ylabel('position')

legend('desired position','actual position')

title('joint6 simulation ')


figure
 plot(t_d,x(7,1:n),t_d,x1(7,1:n))
 xlabel('time')
ylabel('position')

legend('desired position','actual position')

title('joint7 simulation ')
 
 
 
