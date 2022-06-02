clear all

 
traj = 1;
Ts = 1/1250;
a = 0.5;
wt = pi/4;
Tf = 6;
yd=trajectories(traj,wt,a,Ts,Tf);

yd = 1*yd;
posd = yd(1:7,:);
veld = yd(8:14,:);
accd = yd(15:21,:);
t_d = 0:Ts:Tf-Ts;
n1 = length(t_d);
sig = 2e-4;
R = 1e-12*eye(7);
Rp = 1e-3*eye(7);
q1 = 1*eye(7);
q2 = 1e-2*eye(7);
Q = 0*[q1 zeros(7);zeros(7) q2];
H = [eye(7) Ts*eye(7);zeros(7) eye(7)];
I1 = [zeros(7);eye(7)];
I2 = [eye(7);zeros(7)];
P = 1e-10*eye(14);
x1(:,1) = posd(:,1);
x2(:,1) = veld(:,1);
F1 =  diag([0.20519,0.094428,0.094428,0.03,0.001,0.001,0.0003]);
for tk = 1:n1
    x_1(:,tk) = x1(:,tk) + sig*(0.5 - rand(7,1));
    E = posd(:,tk) - x_1(:,tk);
    if tk < n1, Ed = (posd(:,tk+1) - x_1(:,tk)); end
   
    
    K = Ts*I1'*P*inv(Ts^2*P + I1*Rp*I1' + (Ts*I2-I1)*R*(Ts*I2-I1)');
    K1 = 1*K(:,1:7);
    K2 = K(:,8:14);
   
    % Friction:
    %if tk > 1, xx2 =  (x_1(:,tk) -  x_1(:,tk-1))/Ts;else xx2 = zeros(7,1);end
    xx2=veld(:,tk);
    Q22 = diag(diag(bq(xx2)*bq(xx2)'));
    Q = [zeros(7) zeros(7);zeros(7) Q22];
    PHI = H - Ts*I1*K;
    P = PHI*P*PHI' + I1*K*I1*Rp*I1'*K'*I1' + I1*K*(Ts*I2-I1)*R*(Ts*I2-I1)'*K'*I1' + Ts^2*I1*R*I1' + Ts^2*Q;
   
    bf=bq(veld(:,tk));
    sensisitivity=0.1;
    for kkk=1:7;
        if abs(veld(kkk,tk))>sensisitivity
            bfk(kkk)=bf(kkk);
        else 
            bf1=sign(veld(kkk,tk))*bq(sensisitivity);
            bfk(kkk)=bf1(kkk);
        end 
    end 
    
    
    ff = transpose(G(x_1(:,tk)));
   
    if tk == n1, accd(:,tk+1) = accd(:,tk);end
   
    u(:,tk)= ff + (D(x_1(:,tk))+F1)*(accd(:,tk+1) + K1*E + K2*(Ed - E)/Ts);
   
    U = u(:,tk);
   
    
    x1(:,tk+1) =  x2(:,tk)*Ts + x1(:,tk);
    x2(:,tk+1) = (-inv(D(x1(:,tk))+F1)*(transpose(G(x1(:,tk))) + bq(x2(:,tk))) + inv(D(x1(:,tk))+F1)*U)*Ts + x2(:,tk);
    normK1(tk) = norm(K1);
    normK2(tk) = norm(K2);
    E = posd(:,tk) - x_1(:,tk);
    normE(tk) = norm(E);
    normP(tk) = norm(P);
   
end
NormAE = sum(normE)*Ts/Tf
max_std_Torques = max(std(u'))
Uk = u;
figure
subplot(3,1,1),semilogy(normK1),hold,semilogy(normK2,'r'),legend('||K_1||','||K_2||'),grid
subplot(3,1,2),plot(normE),legend('||E||')
subplot(3,1,3),plot(normP),legend('||P||')
figure
for j = 1:7
    subplot(4,2,j),plot(t_d(1:length(Uk)),Uk(j,:)),grid
    title(['Torque: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,yd(j+7,:)*180/pi,t_d,x2(j,1:n1)*180/pi),ylabel('Deg/s')
    title(['Velocity: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,abs( yd(j,:)-x1(j,1:n1) )*180/pi ),ylabel('Deg'),grid
    title(['Position Error: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,yd(j,1:n1)*180/pi,t_d,x1(j,1:n1)*180/pi),ylabel('Deg'),grid
    title(['Position: Joint',num2str(j)])
end
 
 