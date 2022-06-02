clear, close all
load backPk
load backUk
Uk = Uk(1000:length(Uk),:)';
Pk = Pk(1000:length(Pk),:)';
Ts=0.002;
Tf = length(Uk)*Ts;
t_d=0:Ts:Tf-Ts;
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
yd = posd;

accd = [-((pi/4)^2)*sin((pi/4)*t_d);
     -((pi/4)^2)*sin((pi/4)*t_d-pi/2);
     -0.5*((pi/4)^2)*sin((pi/4)*t_d);
     -((pi/4)^2)*sin((pi/4)*t_d+pi/2);
    zeros(3,n1)];

for j = 1:4
    p(j,:) = polyfit(Uk(j,:),accd(j,:),1);
    acc(j,:) = p(j,1)*Uk(j,:) + p(j,2);
end
figure
for j = 1:4
    subplot(2,2,j),plot(t_d(1:length(Uk)),Uk(j,:))
    title(['Torque: Joint',num2str(j)])
end
figure
for j = 1:4
    subplot(2,2,j),plot(t_d,yd(j,:),t_d,Pk(j,1:n1))
    title(['Position: Joint',num2str(j)])
end

figure
for j = 1:4
    subplot(2,2,j),plot(t_d,accd(j,:),t_d(1:length(Uk)),acc(j,:))
    title(['Acceleration: Joint',num2str(j)])
end

