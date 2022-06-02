clear all
arm_home = [ -0.1106   -1.9490   -0.1920    3.1840    0.2825    0.0969   -0.2798];
close all
count=0;
n = 14;p = 7; q = 14;
Ts = 1/1250;
t_d = 0:Ts:5e3*Ts;
count=0;
count1=0;
n1=length(t_d);
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
yd = [posd;veld];
alfa = 1;
yd = alfa*yd;
t = 0:n1-1;
t = Ts*t;
%initial conditions
x1 = alfa*posd(:,1);
x2 = alfa*veld(:,1);
U(:,1) = zeros(7,1);
ek1 = zeros(7,n1);
 
A22 = -(1-Ts)*eye(7);

A21 = zeros(7);
Ae = [zeros(7) eye(7);A21 A22];
C= eye(14);
Ce = C;
Iq = eye(q);
Oq = zeros(q);
H = [Iq Oq Oq -Ce*Ae;Iq Oq Oq zeros(q,n);Oq Iq Oq zeros(q,n);zeros(n,q) zeros(n,q) zeros(n,q) Ae];
Ib = zeros(3*q,3*q+n);
for k11 = 1:3*q,Ib(k11,k11) = 1;end
sig = 1e-4;
sigR = sig;
sigQ = 1e-1*Ts;%1e-3;
P = 1e-4*eye(3*q+n);
Q = sigQ^2*eye(n);
X = [x1(:,1)' x2(:,1)']';
Y = C*X;
Yp = Y;
for tk = 1:n1-1
    
    
   %Ac1(:,:,tk) = [zeros(7,7), eye(7);
           % -inv(D(x1(:,tk)))*L(x1(:,tk),accd(:,tk)) , -2*inv(D(x1(:,tk)))*B(x1(:,tk),x2(:,tk))];
     %  Ae=Ac1;
      % H = [Iq Oq Oq -Ce*Ae;Iq Oq Oq zeros(q,n);Oq Iq Oq zeros(q,n);zeros(n,q) zeros(n,q) zeros(n,q) Ae];
        
    eep = (posd(:,tk+1)-x1(:,tk)).*(posd(:,tk+1)-x1(:,tk));
    dR3 = 1e-12*[ones(7,1);2*ones(7,1)];
    dR2 = 1*dR3;
   
    dR1 = 1e3*[1*eep;2*eep] + 1*dR3;
    dRb = [dR1;dR2;dR3];
    Rb = diag(dRb);
   
    Be = 1*[zeros(7,7);inv(D(x1(:,tk)))]; % multiply by Ts?
  
%     A22 = -(1-Ts)*eye(7);
%     for mm = [3,5,6]
%         Bb = Be*U(:,tk);
%         ccc =  (accd(mm,tk) - Bb(7+mm))/veld(mm,tk);
%         if abs(ccc) < 4;
%             A22(mm,mm) = ccc;
%         end
%     end
%     Ae = [zeros(7) eye(7);zeros(7) A22];
%     H = [Iq Oq Oq -Ce*Ae;Iq Oq Oq zeros(q,n);Oq Iq Oq zeros(q,n);zeros(n,q) zeros(n,q) zeros(n,q) Ae];
   
    
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
    normK1(tk) = norm(K1);
    normK2(tk) = norm(K2);
    normK3(tk) = norm(K3);
   
    
    if tk == 1
       
        Ep = yd(:,tk)-Yp ;
        U(:,tk+1) = U(:,tk) + K1*Ep ;
        Ym = Y;
        Xm = X;
        Y = Yp;
        x1(:,tk+1) =  x2(:,tk)*Ts + x1(:,tk);
        x2(:,tk+1) = (-inv(D(x1(:,tk)))*(transpose(G(x1(:,tk)))+bq(x2(:,tk)))+ inv(D(x1(:,tk)))*(U(:,tk)+transpose(G(x1(:,tk)))))*Ts + x2(:,tk);
        x_1(:,tk) = x1(:,tk) + sig*(0.5 - rand(7,1));
        x_1(:,tk+1) = x1(:,tk+1) + sig*(0.5 - rand(7,1));
        XX2 = zeros(7,1);(x_1(:,tk+1)-x_1(:,tk))/Ts;
        %x_2(:,tk+1)= (-inv(D(x_1(:,tk)))*(transpose(G(x_1(:,tk)))+bq(x2(:,tk)))+ inv(D(x_1(:,tk)))*(U(:,tk)+transpose(G(x_1(:,tk)))))*Ts + XX2 + sigQ*randn(7,1);
       
        Ek(tk) = norm(Ep);
    elseif tk==2
        x_2c=(posd(:,tk+1)-x_1(:,tk))/Ts;
        Yp= [x_1(:,tk)' x_2c']';
        E = yd(:,tk-1) - Y ;
        Ep = yd(:,tk)-Yp ;
        U(:,tk+1) = U(tk-1) + K1*Ep + K2*E;
        Ym = Y;
        Xm = X;
        Y = Yp;
        x1(:,tk+1) =  x2(:,tk)*Ts + x1(:,tk);
        x2(:,tk+1) = (-inv(D(x1(:,tk)))*(transpose(G(x1(:,tk)))+bq(x2(:,tk)))+ inv(D(x1(:,tk)))*(U(:,tk)+transpose(G(x1(:,tk)))))*Ts + x2(:,tk);
        x_1(:,tk) = x1(:,tk) + sig*(0.5 - rand(7,1));
        x_1(:,tk+1) = x1(:,tk+1) + sig*(0.5 - rand(7,1));
        %XX2 = zeros(7,1);(x_1(:,tk+1)-x_1(:,tk))/Ts;
        %x_2(:,tk+1)= (-inv(D(x_1(:,tk)))*(transpose(G(x_1(:,tk)))+bq(x_2(:,tk)))+ inv(D(x_1(:,tk)))*(U(:,tk)+transpose(G(x_1(:,tk)))))*Ts + XX2 + sigQ*randn(7,1);
       
        Ek(tk) = norm(E);
    else
        x_2c=(posd(:,tk+1)-x_1(:,tk))/Ts;
        %x_2c=x2(:,tk);
        Yp= [x_1(:,tk)' x_2c']';
        Y=[x_1(:,tk-1)' ((x_1(:,tk)-x_1(:,tk-1))/Ts)']';
        Ym=[x_1(:,tk-2)' ((x_1(:,tk-1)-x_1(:,tk-2))/Ts)']';
        Em = yd(:,tk-2) - Ym ;
        E = yd(:,tk-1) - Y ;
        Ep = yd(:,tk)-Yp ;
        U(:,tk+1) = U(:,tk-1) + K1*Ep + K2*E + K3*Em;
        Ym = Y;
        Y = Yp;
        x1(:,tk+1) =  x2(:,tk)*Ts + x1(:,tk);
        x2(:,tk+1) = (-inv(D(x1(:,tk)))*(transpose(G(x1(:,tk)))+bq(x2(:,tk)))+ inv(D(x1(:,tk)))*(U(:,tk)+transpose(G(x1(:,tk)))))*Ts + x2(:,tk);
       
        
        x_1(:,tk+1) = x1(:,tk+1) + sig*(0.5 - rand(7,1));
        %XX2 = (x_1(:,tk+1)-x_1(:,tk))/Ts;
        % x_2(:,tk+1)= (-inv(D(x_1(:,tk)))*(transpose(G(x_1(:,tk)))+bq(x_2(:,tk)))+ inv(D(x_1(:,tk)))*(U(:,tk)+transpose(G(x_1(:,tk)))))*Ts + XX2 + sigQ*randn(7,1);
        Ek(tk) = norm(E);
       
    end
   
    normP(tk) = norm(P);
   
end
% figure
% subplot(2,1,1),plot(Ek)
% subplot(2,1,2),plot(normP)
figure
for j = 1:7
    subplot(4,2,j),plot(yd(j,:)),hold,plot(x1(j,1:n1))
    title(['Position: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,yd(j+7,:))
    title(['Velocity: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,yd(j+7,:),t_d,x2(j,1:n1))
    title(['Velocity: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d(1:length(U)),U(j,:)),grid
    title(['Torque: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,yd(j,:),t_d,x1(j,1:n1))
    title(['Position: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,yd(j,:)-x1(j,1:n1))
    title(['Position ERROR: Joint',num2str(j)])
end
figure
subplot(2,1,1),plot(normK1),hold,plot(normK2,'k'),plot(normK3,'g')
legend('||K_1||','||K_2||','||K_3||'),xlabel('sec'),grid
subplot(2,1,2),plot(Ek),grid
 
 
 
 
 
 
