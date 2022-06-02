clear all
load tau
alfa =1;
Ts = 0.002;
Tf = 5; % Final run time
 
t_d = 0:Ts:Tf;
t_d1=0:Ts:Tf+2;
n1 = length(t_d1);
posd = [0.6*t_d + exp(-0.6*t_d)-1.1106;
         0.6*t_d + exp(-0.6*t_d)- 2.9490;
         0.5*t_d + exp(-0.5*t_d)-1-0.192;
        -0.5*t_d - exp(-0.5*t_d)+4.1840;
        -0.5*t_d - exp(-0.5*t_d)+(1+0.2825);
        -0.5*t_d - exp(-0.5*t_d)+(1+0.0969);
        -0.5*t_d - exp(-0.5*t_d)+(1-0.2798)];
    
     veld= [0.6-0.6*(exp(-0.6*t_d));
        0.6-0.6*(exp(-0.6*t_d));
       0.5-0.5*(exp(-0.5*t_d)) ;
        -0.5+0.5*exp(-0.5*t_d);
        -0.5+0.5*exp(-0.5*t_d);
        -0.5+0.5*exp(-0.5*t_d);
        -0.5+0.5*exp(-0.5*t_d)];
    posd=[[-0.1106 -1.9490 -0.192 3.1840 0.2825 0.0969 -0.2798]'*ones(1,1000),posd];
    veld=[zeros(7,1000),veld];
x1 = alfa*posd(:,1);
x2 = alfa*veld(:,1);
yd = [posd;veld];
for tk = 1:n1-1
x1(:,tk+1) =  x2(:,tk)*Ts + x1(:,tk);
       x3=(-inv(D(x1(:,tk)))*(transpose(G(x1(:,tk))))+ inv(D(x1(:,tk)))*Uk(:,tk));
        x2(:,tk+1) = x2(:,tk) + Ts*x3;
end 

figure
for j = 1:7
    subplot(4,2,j),plot(t_d1(1:length(Uk)),Uk(j,:))
    title(['Torque: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d1,yd(j,:),t_d1,x1(j,1:n1))
    title(['Position: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d1,yd(j+7,:),t_d1,x_2(j,1:n1))
    title(['Velocity: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d1,abs( yd(j,:)-x1(j,1:n1) ) )
    title(['Position Error: Joint',num2str(j)])
end