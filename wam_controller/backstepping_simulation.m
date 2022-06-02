clear all

Ts=0.002;
t_d=0:Ts:2;

fs=500;
n = length(t_d);

 x1 = [[-0.11060;-1.9490;-0.19200;3.1840;0.28250;0.09690;-0.27980]*ones(1,n-1)];
 x2=zeros(7,n);
x= [0.5*t_d + exp(-0.5*t_d)-1.1106;
    0.8*t_d + exp(-0.8*t_d)- 2.9490;
    -0.192*ones(1,length(t_d));
   -0.7*t_d - exp(-0.7*t_d)+4.1840];

n = length(t_d);

x=[x;0.2825*ones(1,n);0.0969*ones(1,n);-0.2798*ones(1,n)];

xd= [0.5-0.5*(exp(-0.5*t_d));
0.8-0.8*(exp(-0.8*t_d));
0*ones(1,length(t_d));
 -0.7+0.7*exp(-0.7*t_d);
zeros(3,n)];

xdd=[0.25*(exp(-0.5*t_d));
    0.64*(exp(-0.8*t_d));
    0*ones(1,length(t_d));
    -0.49*exp(-0.7*t_d);
    zeros(3,n)];

K1=diag([30,30,30,30,30,30,30]);
K2=K1;
Angle1=-0.11060;
X = [Angle1 0]';

 for tk = 1:n
    

u(:,tk)= transpose(G(x1(:,tk)))+ D(x1(:,tk))*(xdd(:,tk)-(K1+K2)*([x2(:,tk)]-xd(:,tk))-(eye(7)+K1*K2)*(x1(:,tk)-x(:,tk)));

x1(:,tk+1) =  x2(:,tk)*Ts + x1(:,tk);
            
 x2(:,tk+1) = (-inv(D(x1(:,tk)))*(transpose(G(x1(:,tk))))+ inv(D(x1(:,tk)))*u(:,tk))*Ts + x2(:,tk);
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
 
 
 
