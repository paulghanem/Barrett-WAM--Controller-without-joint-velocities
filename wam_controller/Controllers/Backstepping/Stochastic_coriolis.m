clear all
close all

arm_home =[  -0.1106   -1.9490   -0.1920    3.1840    0.2825    0.0969   -0.2798];
count=0;
count1=0;
traj = 1;
Ts = 1/500;
wt = pi/4;
Tf = 5;
t_d = 0:Ts:Tf-Ts;
n1 = length(t_d);
for k = 3
    a1 = pi/3;
    a2 = 2*pi/2;
     posd(k,:) = a1*(1-cos(a2*t_d))+arm_home(k);
    veld(k,:) = a1*a2*sin(a2*t_d);
    accd(k,:) = a1*a2*a2*cos(a2*t_d);
end
for k = [5 6 7]
    a1 = 2*pi/3;
    a2 = pi/2;
    if k==6 
        a1=pi/2;
    end 
    posd(k,:) = a1*(1-cos(a2*t_d))+arm_home(k);
    veld(k,:) = a1*a2*sin(a2*t_d);
    accd(k,:) = a1*a2*a2*cos(a2*t_d);
    for tk = 1:2/Ts
        if k==5
    if posd(5,tk) > 0.71
    if count1 < 1
        tk1=tk*Ts
        count1=count1+1;
        t_d1=0:Ts:(length(t_d(tk:n1))-1)*Ts;
        posd(k,tk:n1) = a1*(cos(a2*t_d1)-(1-0.71/a1));
    veld(k,tk:n1) = -a1*a2*sin(a2*t_d1);
    accd(k,tk:n1) = -a1*a2*a2*cos(a2*t_d1);
    end 
    end
        end 
    if k==6
    if posd(k,tk) > 1.5
    if count < 1
        tk2=tk*Ts
        count=count+1;
        t_d1=0:Ts:(length(t_d(tk:n1))-1)*Ts;
        posd(k,tk:n1) = a1*(cos(a2*t_d1)-(1-1.5/a1));
    veld(k,tk:n1) = -a1*a2*sin(a2*t_d1);
    accd(k,tk:n1) = -a1*a2*a2*cos(a2*t_d1);
    end 
    end 
   
    
        
    
    end
    end 
end
% for k = 5
%     a1 = pi/8;
%     a2 = 2*pi/25;
%      posd(k,:) = a1*(1-cos(a2*t_d));
%     veld(k,:) = a1*a2*sin(a2*t_d);
%     accd(k,:) = a1*a2*a2*cos(a2*t_d);
% end
% for k = 6
%     a1 = -pi/12;
%     a2 = 2*pi/30;
%      posd(k,:) = a1*(1-cos(a2*t_d));
%     veld(k,:) = a1*a2*sin(a2*t_d);
%     accd(k,:) = a1*a2*a2*cos(a2*t_d);
% end
% for k = 7
%     a1 = pi/4;
%     a2 = 2*pi/10;
%      posd(k,:) = a1*(1-cos(a2*t_d));
%     veld(k,:) = a1*a2*sin(a2*t_d);
%     accd(k,:) = a1*a2*a2*cos(a2*t_d);
% end
for k = [1 2 4]
    a1 = pi/3;
    a2 = pi/2;
    if k==4 
    posd(k,:) = -a1*(1-cos(a2*t_d))+arm_home(k);
    veld(k,:) = -a1*a2*sin(a2*t_d);
    accd(k,:) = -a1*a2*a2*cos(a2*t_d);
    else 
    
    posd(k,:) = a1*(1-cos(a2*t_d))+arm_home(k);
    veld(k,:) = a1*a2*sin(a2*t_d);
    accd(k,:) = a1*a2*a2*cos(a2*t_d);
    end 
end
yd = [posd ; veld];
n1 = length(t_d);
sig = 2e-4;
ccc = 1e-14;
R = ccc*eye(7);
Rp = 1e-3*eye(7);
q1 = 1*eye(7);
q2 = 1e-2*eye(7);
Q = 0*[q1 zeros(7);zeros(7) q2];
H = [eye(7) Ts*eye(7);zeros(7) eye(7)];
I1 = [zeros(7);eye(7)];
I2 = [eye(7);zeros(7)];
P = 1e-10*eye(14);
x1(:,1) = posd(:,1);
x2(:,1) = zeros(7,1);
F1 =  diag([0.20519,0.094428,0.094428,0.03,0.001,0.001,0.0003]);
for tk = 1:n1
    x_1(:,tk) = x1(:,tk) + sig*(0.5 - rand(7,1));
    E = posd(:,tk) - x_1(:,tk);
    if tk < n1, Ed = (posd(:,tk+1) - x_1(:,tk)); end
  
    
    K = Ts*I1'*P*inv(Ts^2*P + I1*Rp*I1' + (Ts*I2-I1)*R*(Ts*I2-I1)');
    K1 = K(:,1:7);
    K2 = K(:,8:14);
  
    % Friction:
  
    xx2=veld(:,tk);
    Q22 = diag(diag(bq(xx2)*bq(xx2)'));
    Q = [zeros(7) zeros(7);zeros(7) Q22];
  
    PHI = H - Ts*I1*K;
    P = PHI*P*PHI' + I1*K*I1*Rp*I1'*K'*I1' + I1*K*(Ts*I2-I1)*R*(Ts*I2-I1)'*K'*I1' + Ts^2*I1*R*I1' + 1*Ts^2*Q;
  
    bf(:,tk)=bq(veld(:,tk));
    sensisitivity = 0.2;
    for kkk= 1:7
        if abs(veld(kkk,tk))>sensisitivity
            bfk(kkk,tk)=bf(kkk,tk);%,abs(veld(:,tk)),pause
        else
            bfk(kkk,tk)=0;
        end
    end
  
  
    %bfk = 0*bfk;
    ff = transpose(G(x_1(:,tk)));%+bfk(:,tk);
   
   
    if tk == n1, accd(:,tk+1) = accd(:,tk);end
  
    u(:,tk)= ff +  (D(x_1(:,tk))+F1)*(accd(:,tk) + K1*E + K2*(Ed - E)/Ts);
    ud(:,tk)= transpose(G(posd(:,tk))) + bq(veld(:,tk)) + (D(posd(:,tk))+F1)*(accd(:,tk+1));
    U = u(:,tk);
  
    
    x1(:,tk+1) =  x2(:,tk)*Ts + x1(:,tk);
    x2(:,tk+1) = (-inv(D(x1(:,tk))+F1)*(transpose(G(x1(:,tk)))+transpose(Co(x1(:,tk),x2(:,tk))) + bq(x2(:,tk))) + inv(D(x1(:,tk))+F1)*U)*Ts + x2(:,tk);
   
    normK1(tk) = norm(K1);
    normK2(tk) = norm(K2);
    E = posd(:,tk) - x_1(:,tk);
    normE(tk) = norm(E);
    normP(tk) = norm(P);
  
end
NormAE = sum(normE)*Ts/Tf
max_std_Torques = max(std(ud'-u'))
Uk = u;
figure
subplot(3,1,1),semilogy(normK1),hold,semilogy(normK2,'r'),legend('||K_1||','||K_2||'),grid
subplot(3,1,2),plot(normE),legend('||E||')
subplot(3,1,3),plot(normP),legend('||P||')
figure
for j = 1:7
    subplot(4,2,j),plot(t_d(1:length(Uk)),Uk(j,:),'r'),hold,plot(t_d(1:length(Uk)),ud(j,:)), grid
    title(['Torque: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,yd(j+7,:)*180/pi,t_d,x2(j,1:n1)*180/pi),ylabel('Deg/s')
    title(['Velocity: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,abs( yd(j,:)-x1(j,1:n1) ) ),ylabel('rad'),grid
    title(['Position Error: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,abs( veld(j,:)-x2(j,1:n1) ) ),ylabel('rad/s'),grid
    title(['Velocity Error: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,yd(j,1:n1)*180/pi,t_d,x1(j,1:n1)*180/pi),ylabel('Deg'),grid
    title(['Position: Joint',num2str(j)])
end
 
 
 figure
for j = 1:7
    subplot(4,2,j),plot(x2(j,1:n1),bf(j,:),'r',x2(j,1:n1),bfk(j,:),'g'),ylabel('N.m'),xlabel('rad/s'),grid
    title(['friction : Joint',num2str(j)])
  
end

 
 




