clear
close all
count=0;
n = 14;p = 7; q = 7;
Ts = 0.002;
Tf = 1; % Final run time
 
t_d = 0:Ts:Tf;
t_d1=0:Ts:Tf;
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
    %posd=[[-0.1106 -1.9490 -0.192 3.1840 0.2825 0.0969 -0.2798]'*ones(1,1000),posd];
    %veld=[zeros(7,1000),veld];
yd = [posd];
alfa = 1;
yd = alfa*yd;
 
t = 0:n1-1;
t = Ts*t;
%initial conditions
x1 = alfa*posd(:,1);
x2 = alfa*veld(:,1);
U = zeros(7,1);
 
Ae = 0.9*eye(n);
C= [eye(7),zeros(7,7)];

Ce = C;
Iq = eye(q);
Oq = zeros(q);
H = [Iq Oq Oq -Ce*Ae;Iq Oq Oq zeros(q,n);Oq Iq Oq zeros(q,n);zeros(n,q) zeros(n,q) zeros(n,q) Ae];
Ib = zeros(3*q,3*q+n);
for k11 = 1:3*q,Ib(k11,k11) = 1;end
sig = 1e-4;
sigR = sig;
sigQ = 1e-3;
P = 1e2*eye(3*q+n);
Rb = 1e-10*diag(1*ones(3*q,1));
Q = 0*sigQ^2*eye(n);
X = [x1(:,1)' x2(:,1)']';
Y = C*X;
Yp = Y;
x_1(:,1) = x1(:,1);
 
 
for tk = 1:n1-1
    Uk(:,tk) = U;
   x3(:,tk)=(-inv(D(x1(:,tk))+diag([0.20519,0.094428,0.094428,0.03,0.001,0.001,0.0003]))*(transpose(G(x1(:,tk)))+bq(x2(:,tk)))+ inv(D(x1(:,tk))+diag([0.20519,0.094428,0.094428,0.03,0.001,0.001,0.0003]))*U);
   %Ac1=[zeros(7,7), eye(7);
            %-inv(D(x1(:,tk))+diag([0.20519,0.094428,0.094428,0.03,0.001,0.001,0.0003]))*L(x1(:,tk),x3(:,tk)) , zeros(7,7)];
    
   Bc=[zeros(7,7);inv(D(x1(:,tk))+diag([0.20519,0.094428,0.094428,0.03,0.001,0.001,0.0003]))];
   
   
       sysc=ss(Ae,Bc,C,0);
       sys=c2d(sysc,Ts);
       [A,B,C,D1]=ssdata(sys);
       Ae=A;
       Be=B;
       Ce=C;
    Delyd = [(yd(:,tk+1)-yd(:,tk))];
    Del2yd = Delyd*Delyd';
   
    Gd = [Del2yd Oq Oq zeros(q,n); Oq Oq Oq zeros(q,n); Oq Oq Oq zeros(q,n);zeros(n,q) zeros(n,q) zeros(n,q) Q];
   
    
    
    EE = [Ce*Be;zeros(q,p);zeros(q,p);-Be];
    K = inv(EE'*EE)*EE'*H*P*Ib'*inv(Ib*P*Ib' + Rb);
    PHI = H - EE*K*Ib;
    %if max(abs(eig(PHI))) > 1, disp('max(abs(eig(PHI))) > 1'),tk, pause(1),end
   
    P = PHI*P*PHI' + EE*K*Rb*K'*EE' + Gd;
    K1 = K(:,1:q);
    K2 = K(:,q+1:2*q);
    K3 = K(:,2*q+1:3*q);
    %     normK1(tk) = norm(K1);
    %     normK2(tk) = norm(K2);
    %     normK3(tk) = norm(K3);
   
    
    if tk == 1
        E = yd(:,tk) - Y ;
        Ep = yd(:,tk+1)-Yp ;
        U = U + K1*Ep + K2*E;
        Ym = Y;
        Y = Yp;
       
        x_1(:,tk) = x1(:,tk) ;
        % Modeling the robot
        x1(:,tk+1) =  x2(:,tk)*Ts + x1(:,tk);
        x2(:,tk+1) = (-inv(D(x1(:,tk)))*(transpose(G(x1(:,tk))))+ inv(D(x1(:,tk)))*U)*Ts + x2(:,tk);
        %
       
    else
        Em = yd(:,tk-1) - Ym ;
        E = yd(:,tk) - Y ;
        Ep = yd(:,tk+1)-Yp ;
        U = U + K1*Ep + K2*E + K3*Em;
        Ym = Y;
        Y = Yp;
        % Modeling the robot
        x1(:,tk+1) =  x2(:,tk)*Ts + x1(:,tk);
        x2(:,tk+1) = (-inv(D(x1(:,tk))+diag([0.20519,0.094428,0.094428,0.03,0.001,0.001,0.0003]))*(transpose(G(x1(:,tk)))+bq(x2(:,tk)))+ inv(D(x1(:,tk))+diag([0.20519,0.094428,0.094428,0.03,0.001,0.001,0.0003]))*(U+transpose(G(x1(:,tk)))))*Ts + x2(:,tk);
        %
        %
       
        Yp = [x1(:,tk+1)];
    end
    YYY = [x1(:,tk)];
    Ek(tk) = max(abs(yd(:,tk) - YYY));
    normP(tk) = norm(P);
    normK1(tk) = norm(K1);
    normK2(tk) = norm(K2);
    normK3(tk) = norm(K3);
    
   
end
 plot(normK1),hold,plot(normK2,'k'),plot(normK3,'g')
legend('||K_1||','||K_2||','||K_3||'),xlabel('sec'),grid
figure
subplot(2,1,1),plot(t(1:length(Ek)),Ek),ylabel('max(|E|)')
subplot(2,1,2),plot(t(1:length(normP)),normP),ylabel('||P||')
xlabel('sec')
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
    subplot(4,2,j),plot(t_d1,x2(j,1:n1))
    title(['Velocity: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d1,abs( yd(j,:)-x1(j,1:n1) ) )
    title(['Position Error: Joint',num2str(j)])
end
 
