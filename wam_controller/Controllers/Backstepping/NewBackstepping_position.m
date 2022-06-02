clear all
close all
Ts=0.002;
Tf=5;
t_d=0:Ts:Tf-Ts;
n1=length(t_d);
wt=pi/4;%angular velocity
traj=1;%0 ==> sinusoidal 4dof,1==>sinusoidal 7dof,2==> exponential 7dof
a=0.5;%exponential coeficient

yd=trajectories(traj,wt,a,Ts,Tf);
posd=yd(1:7,:);
veld=yd(8:14,:);
accd=yd(15:21,:);
sig = 1e-3;
x1(:,1) = posd(:,1);
x2(:,1) = veld(:,1);
u(:,1) = zeros(1,7);
C = [eye(7) zeros(7)];
F1 =  diag([0.20519,0.094428,0.094428,0.03,0.001,0.001,0.0003]);


for tk = 1:n1
    x_1(:,tk) = x1(:,tk) + sig*(0.5 - rand(7,1));
    if tk > 1
        x_2(:,tk) = (x1(:,tk) - x1(:,tk-1))/Ts;
    elseif tk == 1
        x_2(:,tk) = x2(:,tk);
    end
  E=posd(:,tk)-x_1(:,tk);
  K1=-2e4*eye(7)*diag(abs(E));
   
    % K = -3e3*eye(7)/log10(tk+9);
    
    %K1 = K(:,1:7);
  
   ff = (transpose(G(x_1(:,tk))));
    u(:,tk)= ff + (D(x_1(:,tk))+F1)*(accd(:,tk)+(eye(7)+K1)*(x_1(:,tk)-posd(:,tk)));
    x1(:,tk+1) =  x2(:,tk)*Ts + x1(:,tk);
  
    %x2(:,tk+1) = (-inv(D(x1(:,tk)))*(transpose(G(x1(:,tk))))+ inv(D(x1(:,tk)))*u(:,tk))*Ts + x2(:,tk);
    x2(:,tk+1) = (-inv(D(x1(:,tk))+F1)*(transpose(G(x1(:,tk)))+bq(x2(:,tk)))+ inv(D(x1(:,tk))+F1)*u(:,tk))*Ts + x2(:,tk);
    normK1(tk) = norm(K1);
   
   if tk < n1, u(:,tk+1) = u(:,tk);end
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
figure
plot(normK1)
 
 
