clear
close all
count=0;
n = 14;p = 7; q = 14;
Ts = 0.002;
t_d = 0:Ts:4e2*Ts;
n1 = length(t_d);
KF = 1; % KF = 1 with KF, and KF = 0 without KF
Trajexp = 0; % Trajexp = 1 exp reference trajectories and Trajexp = 0 sin reference traj
if Trajexp == 1
    veld =[0.2-0.2*(exp(-0.2*t_d));
        0.3-0.3*(exp(-0.3*t_d));
        0*ones(1,length(t_d));
        -0.3+0.3*exp(-0.3*t_d)];
    posd = [0.2*t_d + exp(-0.2*t_d)-1.1106;
        0.3*t_d + exp(-0.3*t_d)- 2.9490;
        -0.192*ones(1,length(t_d));
        -0.3*t_d - exp(-0.3*t_d)+4.1840];
    veld=[veld;zeros(3,n1)];
   
    posd=[posd;0.2825*ones(1,n1);0.0969*ones(1,n1);-0.2798*ones(1,n1)];
elseif Trajexp == 0
   
    posd = [-0.1106+sin((pi/4)*t_d);
        -0.932+sin((pi/4)*t_d-pi/2);
        -0.1920+0.5*sin((pi/4)*t_d);
        2.1+sin((pi/4)*t_d+pi/2);
        0.2825*ones(1,n1);
        0.0969*ones(1,n1);
        -0.2798*ones(1,n1)];
   
    
    veld = [(pi/4)*cos((pi/4)*t_d);
        (pi/4)*cos((pi/4)*t_d-pi/2);
        0.5*(pi/4)*cos((pi/4)*t_d);
        (pi/4)*cos((pi/4)*t_d+pi/2);
        zeros(3,n1)];
   
end
 
yd = [posd;veld];
alfa = 1;
yd = alfa*yd;
 
t = 0:n1-1;
t = Ts*t;
%initial conditions
 
normE = 1000
for r1 = [5000 7000 10000]
    for q1 = [10 100 1000 ]
        for q2 = [ 1e-2 1e-3 1e-4]
            for r2 = [1e-1 1 10]
                for r3 = [1e-1 1e-2 1e-3]
                    for rv = [ 1e-7 1e-10 ]
                        x1 = alfa*posd(:,1);
                        x2 = alfa*veld(:,1);
                       
                        U = zeros(7,1);
                       
                        
                        
                        Ae = (1-Ts)*eye(n);
                       
                        C= eye(14);
                        Ce = C;
                        Iq = eye(q);
                        Oq = zeros(q);
                        H = [Iq Oq Oq -Ce*Ae;Iq Oq Oq zeros(q,n);Oq Iq Oq zeros(q,n);zeros(n,q) zeros(n,q) zeros(n,q) Ae];
                       
                        Ib = zeros(3*q,3*q+n);
                        for k11 = 1:3*q,Ib(k11,k11) = 1;end
                        sig = 1e-4;
                        sigR = sig;
                        sigQ = 1e-3;
                        P = 1e-8*eye(3*q+n);
                        if KF == 1
                            rbb1 = 4e-7*ones(1,q/2);
                            rbb2 = rv*ones(1,q/2);
                            % r1 = 2000;
                            %                  r2 = 1e-3;
                            %                  r3 = 1e-3;
                            rbb12 = [rbb1 rbb2];
                            rbb = [r1*rbb12 r2*rbb12 r3*rbb12];
                            Rb = 1*diag(rbb);
                        elseif KF == 0
                            rbb1 = sig^2*ones(1,q/2);
                            rbb2 = 1*sig^2*ones(1,q/2);
                            rbb12 = [rbb1 rbb2];
                            rbb = [r1*rbb12 r2*rbb12 r3*rbb12];
                            Rb = 1*diag(rbb);
                        end
                       
                        
                        Q = 1*sigQ^2*eye(n);
                        X = [x1(:,1)' x2(:,1)']';
                        Y = C*X;
                        Yp = Y;
                        x_1(:,1) = x1(:,1);
                        % Parameters for KF
                        %r1 = sig;
                        Rkf = 1e-6;
                        %Qkf = 5e20;%
                       
                        %              q1 = 1;
                        %              q2 = 1e-3;
                        Qkf = [q1 q1/20;q2/20 q2];
                        Akf = [1 Ts;0 1];
                        %Bkf = [2e-6 2e-3]';
                        Bkf = [2e-3 2e-6;0 1];
                        Ckf = [1 0];
                        Pkf = 1e-8*eye(2);
                       
                        % Steady state KF
                        % for k = 1:100
                        %     Pm = Akf*Pkf*Akf' + Bkf*Qkf*Bkf';
                        %     Kkf = Pm*Ckf'*inv(Ckf*Pm*Ckf' + Rkf);
                        %     Pkf = (eye(2) - Kkf*Ckf)*Pm;
                        %     Kk1(k) = Kkf(1);
                        %     Kk2(k) = Kkf(2);
                        % end
                       
                        Xkfp = [x1(:,1) x2(:,1)]; %zeros(7,2);
                       
                        for tk = 1:n1-1
                            Uk(:,tk) = U;
                            Be = Ts*[zeros(7,7);inv(D(x_1(:,tk)))];
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
                               
                                x_1(:,tk) = x1(:,tk) + sig*(0.5 - rand(7,1));
                                % Modeling the robot
                                x1(:,tk+1) =  x2(:,tk)*Ts + x1(:,tk);
                                x2(:,tk+1) = (-inv(D(x1(:,tk)))*(transpose(G(x1(:,tk))))+ inv(D(x1(:,tk)))*U)*Ts + x2(:,tk);
                                %
                                x_1(:,tk+1) = x_1(:,tk); %Since at t=0, velocity is zero
                                XX2 = zeros(7,1);
                                x_2(:,tk+1)= (-inv(D(x_1(:,tk)))*(transpose(G(x_1(:,tk))))+ inv(D(x_1(:,tk)))*U)*Ts + XX2 + sigQ*randn(7,1);
                                Yp = [x_1(:,tk+1)' x_2(:,tk+1)']';
                               
                            else
                                Em = yd(:,tk-1) - Ym ;
                                E = yd(:,tk) - Y ;
                                Ep = yd(:,tk+1)-Yp ;
                                U = U + K1*Ep + K2*E + K3*Em;
                                Ym = Y;
                                Y = Yp;
                                % Modeling the robot
                                x1(:,tk+1) =  x2(:,tk)*Ts + x1(:,tk);
                                x2(:,tk+1) = (-inv(D(x1(:,tk)))*(transpose(G(x1(:,tk))))+ inv(D(x1(:,tk)))*U)*Ts + x2(:,tk);
                                %
                                pos_meas = x1(:,tk) + sig*(0.5 - rand(7,1)); % In experimental work you do not add noise instead you only use measurement of position
                                x_1(:,tk+1) =  x_2(:,tk)*Ts + pos_meas; % Here we are estimating position using the extimate of x2 and measurement of x1 at present time
                               
                                XX2 = (x_1(:,tk+1)-x_1(:,tk))/Ts; %
                                if KF == 1
                                    % Real time KF
                                    Pm = Akf*Pkf*Akf' + Bkf*Qkf*Bkf';
                                    Kkf = Pm*Ckf'*inv(Ckf*Pm*Ckf' + Rkf);
                                    Pkf = (eye(2) - Kkf*Ckf)*Pm;
                                    for kk = 1:7
                                        Xkfm(kk,:) = Akf*Xkfp(kk,:)'; %[x_1(kk,tk+1) Xp(2)];
                                        yk = x_1(kk,tk+1) - Xkfm(kk,1);
                                        %yk = pos_meas(kk) - Xkfm(kk,1);
                                        Xkfp(kk,:) = Xkfm(kk,:) + (Kkf*yk)';
                                    end
                                    x_2(:,tk+1)= (-inv(D(pos_meas))*(transpose(G(pos_meas)))+ inv(D(pos_meas))*U)*Ts + Xkfm(:,2) + sigQ*randn(7,1);
                                    Yp = [Xkfp(:,1)' Xkfp(:,2)']';
                                    % Yp = [Xkfp(:,1)' x2(:,tk+1)']';
                                    % x2(:,tk+1) - Xkfp(:,2),pause
                                elseif KF == 0
                                    x_2(:,tk+1)= (-inv(D(pos_meas))*(transpose(G(pos_meas)))+ inv(D(pos_meas))*U)*Ts + XX2 + sigQ*randn(7,1);
                                    Yp = [x_1(:,tk+1)' x_2(:,tk+1)']';
                                end
                               
                                
                                
                            end
                            YYY = [x1(:,tk);x2(:,tk)];
                            Ek(tk) = max(abs(yd(:,tk) - YYY));
                            normP(tk) = norm(P);
                        end
                       
                        
                        %plot(Ek),pause
                       
                        
                        if max(Ek) < normE
                            %
                            r1o = r1;
                            r2o = r2;
                            r3o = r3;
                            q1o = q1;
                            q2o = q2;
                            rvo = rv;
                            normE = max(Ek)
                            XXX1 = x1;
                            XXX2 = x2;
                        end
                       
                        
                    end
                end
            end
        end
    end
end
x1 = XXX1;
x2 = XXX2;
rs = [r1o r2o r3o]
qs = [q1o q2o]
rvo
% plot(normK1),hold,plot(normK2,'k'),plot(normK3,'g')
% legend('||K_1||','||K_2||','||K_3||'),xlabel('sec'),grid
% figure
% subplot(2,1,1),plot(Ek)
% subplot(2,1,2),plot(normP)
figure
for j = 1:7
    subplot(4,2,j),plot(t_d(1:length(Uk)),Uk(j,:))
    title(['Torque: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,yd(j+7,:))
    title(['Velocity: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,yd(j,:),t_d,x1(j,1:n1))
    title(['Position: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,yd(j+7,:),t_d,x2(j,1:n1))
    title(['Velocity: Joint',num2str(j)])
end
 
 
