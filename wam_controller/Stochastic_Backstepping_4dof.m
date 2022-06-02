
clear all
close all
arm_home =[  -0.1106   -1.9490   -0.1920    3.1840  ];
count=0;
count1=0;
traj = 1;
Ts = 1/5000;
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
R = ccc*eye(4);
Rp = 1e-3*eye(4);
q1 = 1*eye(4);
q2 = 1e-2*eye(4);
Q = 0*[q1 zeros(4);zeros(4) q2];
H = [eye(4) Ts*eye(4);zeros(4) eye(4)];
I1 = [zeros(4);eye(4)];
I2 = [eye(4);zeros(4)];
P = 1e-10*eye(8);
x1(:,1) = posd(:,1);
x2(:,1) = zeros(4,1);
F1 =  diag([0.20519,0.094428,0.094428,0.03]);
for tk = 1:n1
    x_1(:,tk) = x1(:,tk) + sig*(0.5 - rand(4,1));
    E = posd(:,tk) - x_1(:,tk);
    if tk < n1, Ed = (posd(:,tk+1) - x_1(:,tk)); end
  
    
    K = Ts*I1'*P*inv(Ts^2*P + I1*Rp*I1' + (Ts*I2-I1)*R*(Ts*I2-I1)');
    K1 = K(:,1:4);
    K2 = K(:,5:8);
  
    % Friction:
  
    xx2=veld(:,tk);
    Q22 = diag(diag(bq1(xx2)*bq1(xx2)'));
    Q = [zeros(4) zeros(4);zeros(4) Q22];
  
    PHI = H - Ts*I1*K;
    P = PHI*P*PHI' + I1*K*I1*Rp*I1'*K'*I1' + I1*K*(Ts*I2-I1)*R*(Ts*I2-I1)'*K'*I1' + Ts^2*I1*R*I1' + 1*Ts^2*Q;
  
    bf(:,tk)=bq1(veld(:,tk));
    sensisitivity = 0.2;
    for kkk= 1:4
        if abs(veld(kkk,tk))>sensisitivity
            bfk(kkk,tk)=bf(kkk,tk);%,abs(veld(:,tk)),pause
        else
            bfk(kkk,tk)=0;
        end
    end
  
  
    %bfk = 0*bfk;
    ff = transpose(G1(x_1(:,tk)));%+bfk(:,tk);
   
   
    if tk == n1, accd(:,tk+1) = accd(:,tk);end
  
    u(:,tk)= ff + (D1(x_1(:,tk))+F1)*(accd(:,tk+1) + K1*E + K2*(Ed-E)/Ts);
    ud(:,tk)= transpose(G1(posd(:,tk))) + bq1(veld(:,tk)) + (D1(posd(:,tk))+F1)*(accd(:,tk+1));
    U = u(:,tk);
  
    
    x1(:,tk+1) =  x2(:,tk)*Ts + x1(:,tk);
    x2(:,tk+1) = (-inv(D1(x1(:,tk))+F1)*(transpose(G1(x1(:,tk))) + bq1(x2(:,tk))) + inv(D1(x1(:,tk))+F1)*U)*Ts + x2(:,tk);
   
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
for j = 1:4
    subplot(4,2,j),plot(t_d(1:length(Uk)),Uk(j,:),'r'),hold,plot(t_d(1:length(Uk)),ud(j,:)), grid
    title(['Torque: Joint',num2str(j)])
end
figure
for j = 1:4
    subplot(4,2,j),plot(t_d,yd(j+4,:)*180/pi,t_d,x2(j,1:n1)*180/pi),ylabel('Deg/s')
    title(['Velocity: Joint',num2str(j)])
end
figure
for j = 1:4
    subplot(4,2,j),plot(t_d,abs( yd(j,:)-x1(j,1:n1) ) ),ylabel('rad'),grid
    title(['Position Error: Joint',num2str(j)])
end
figure
for j = 1:4
    subplot(4,2,j),plot(t_d,abs( veld(j,:)-x2(j,1:n1) ) ),ylabel('rad/s'),grid
    title(['Velocity Error: Joint',num2str(j)])
end
figure
for j = 1:4
    subplot(4,2,j),plot(t_d,yd(j,1:n1)*180/pi,t_d,x1(j,1:n1)*180/pi),ylabel('Deg'),grid
    title(['Position: Joint',num2str(j)])
end
 
 
 figure
for j = 1:4
    subplot(4,2,j),plot(x2(j,1:n1),bf(j,:),'r',x2(j,1:n1),bfk(j,:),'g'),ylabel('N.m'),xlabel('rad/s'),grid
    title(['friction : Joint',num2str(j)])
  
end

 
 




