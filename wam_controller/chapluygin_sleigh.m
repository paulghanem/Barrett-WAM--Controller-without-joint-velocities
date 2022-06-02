
clear all
clc
Tend=50;
Ts=0.002;
t_d=0:Ts:Tend;
v=0.1;
w=0;
Theta=0;
d = 0.1;                % drag
m = 1;                  % mass
l = 1;                  % length
J = 1;                  % moment of inertia
Jd = J;                 % moment of inertia (disk)


b = J+m*l^2;            % paralell axis theorem

K1=0.1 ;
K2=1;
for tk=1:1:Tend/Ts
    
u(:,tk)=b*(-K1*w(:,tk)-K2*sin(2-Theta(:,tk)));

vd(:,tk)=l*w(:,tk)^2-d*v(:,tk);
Thetad(:,tk)=w(:,tk);
wd(:,tk)=-(m*l/b)*v(:,tk)*w(:,tk)-u(:,tk)/b;






v(:,tk+1)=vd(:,tk)*Ts+v(:,tk);
Theta(:,tk+1)=Thetad(:,tk)*Ts+Theta(:,tk);
w(:,tk+1)=wd(:,tk)*Ts+wd(:,tk);
end 

subplot(4,1,1)
plot(t_d,Theta)
xlabel('t')
ylabel('Theta')

subplot(4,1,2)
plot(t_d,w)
xlabel('t')
ylabel('w')

subplot(4,1,3)
plot(t_d,v)
xlabel('t')
ylabel('v')

subplot(4,1,4)
plot(v,w)
xlabel('v')
ylabel('w')

figure 
plot(t_d,w.^2./v.^2)
xlabel('t')

