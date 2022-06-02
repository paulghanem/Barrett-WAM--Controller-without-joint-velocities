clear all
close all
Ts = 0.002;
Tf = 6;
t_d = 0:Ts:Tf-Ts;
n1 = length(t_d);
posd = [-0.1106+sin((pi/4)*t_d);
    -0.932+sin((pi/4)*t_d-pi/2);
    -0.1920+0.5*sin((pi/4)*t_d);
    2.1+sin((pi/4)*t_d+pi/2)];
posd=[posd;0.2407*ones(1,n1);0.00681*ones(1,n1);-0.1180*ones(1,n1)];
veld= [(pi/4)*cos((pi/4)*t_d);
    (pi/4)*cos((pi/4)*t_d-pi/2);
    0.5*(pi/4)*cos((pi/4)*t_d);
    (pi/4)*cos((pi/4)*t_d+pi/2)];
n1 = length(t_d);
veld=[veld;zeros(3,n1)];
yd = [posd;veld];
accd = [-((pi/4)^2)*sin((pi/4)*t_d);
    -((pi/4)^2)*sin((pi/4)*t_d-pi/2);
    -0.5*((pi/4)^2)*sin((pi/4)*t_d);
    -((pi/4)^2)*sin((pi/4)*t_d+pi/2);
    zeros(3,n1)];
sig = 1e-3;
K1 = 1*diag([30,30,30,30,5,5,5]);
K2 = K1;
Angle1=-0.11060;
X = [Angle1 0]';
x1(:,1) = posd(:,1);
x2(:,1) = veld(:,1);
for tk = 1:n1
    x_1(:,tk) = x1(:,tk) + sig*(0.5 - rand(7,1));
    if tk > 1
        x_2(:,tk) = (x1(:,tk) - x1(:,tk-1))/Ts;
    elseif tk == 1
        x_2(:,tk) = x2(:,tk);
    end
    F1=diag([0.20519,0.094428,0.094428,0.03,0.001,0.001,0.0003]);
    u(:,tk)= transpose(G(x_1(:,tk)))+ (D(x_1(:,tk))+F1)*(accd(:,tk)-(K1+K2)*([x_2(:,tk)]-veld(:,tk))-(eye(7)+K1*K2)*(x_1(:,tk)-posd(:,tk)));
   
    x1(:,tk+1) =  x2(:,tk)*Ts + x1(:,tk);
   
    %x2(:,tk+1) = (-inv(D(x1(:,tk)))*(transpose(G(x1(:,tk))))+ inv(D(x1(:,tk)))*u(:,tk))*Ts + x2(:,tk);
    x2(:,tk+1) = (-inv(D(x1(:,tk))+F1)*(transpose(G(x1(:,tk)))+bq(x2(:,tk)))+ inv(D(x1(:,tk))+F1)*u(:,tk))*Ts + x2(:,tk);
end
Uk = u;
figure
for j = 1:7
    subplot(4,2,j),plot(t_d(1:length(Uk)),Uk(j,:))
    title(['Torque: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,yd(j,1:n1),t_d,x1(j,1:n1))
    title(['Position: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,yd(j+7,:),t_d,x2(j,1:n1))
    title(['Velocity: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,abs( yd(j,:)-x1(j,1:n1) ) )
    title(['Position Error: Joint',num2str(j)])
end
 
 
