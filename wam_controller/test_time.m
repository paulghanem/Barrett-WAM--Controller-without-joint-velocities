
close all 

arm_home =[  -0.1106   -1.9490   -0.1920    3.1840    0.2825    0.0969   -0.2798];
count=0;
count1=0;
traj = 1;
Ts = 1/1000;
wt = pi/4;
Tf = 5;
t_d = 0:Ts:Tf-Ts;
n1 = length(t_d);

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



for tk= 1:200
    t= cputime; 
    Co(x1(:,tk),x2(:,tk));
    e1(tk)=cputime -t; 
    
    t=cputime;
    D(x1(:,tk));
    e2(tk)=cputime-t;
    
    t= cputime ;
    transpose(G(x_1(:,tk)))+transpose(Co(x1(:,tk),x2(:,tk)))+(D(x_1(:,tk))+F1)*(accd(:,tk) + K1*E + K2*(Ed - E)/Ts);
    e3(tk)=cputime -t; 
    
    t=cputime;
    transpose(G(x_1(:,tk)))+(D(x_1(:,tk))+F1)*(accd(:,tk) + K1*E + K2*(Ed - E)/Ts);
    e4(tk)=cputime-t;
    
    t= cputime;
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
  %+bfk(:,tk);
   
   
    if tk == n1, accd(:,tk+1) = accd(:,tk);end
  
      transpose(G(x_1(:,tk)))+  (D(x_1(:,tk))+F1)*(accd(:,tk) + K1*E + K2*(Ed - E)/Ts);
     e5(tk)=cputime-t;
    

end 
