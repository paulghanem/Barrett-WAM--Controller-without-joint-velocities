clear
close all
load qd 
load q
Angle1 = q; % To test Joint n just use instead Angle1 = Joint_Angles(:,n);
fs = 500; % Sample rate

Ac = [0 1;0 0];
Bc = [0 1]';
C = [1 0];
sysc = ss(Ac, Bc, C, 0);
sys = c2d(sysc,1/fs);
[A,B,C,D] = ssdata(sys);
for j=1:7 
    Vel1(:,j) = diff(Angle1(:,j))*fs; % Estimate by direct differencing
t = [0:length(Vel1(:,j))]/fs;
X = [Angle1(1,j) 0]'; % Initial condition (guess)
%Tuning parameters: For example for smoother performance you can decrease the value of Q at the cost of slow response or larger transient response errors
Q = 1000;
R = 1e-6;
P = [1e-4 0;0 1];
% KF loop
for k = 1:length(Angle1(:,j))
    Xm = A*X;
    Pm = A*P*A' + B*Q*B';
    yk = Angle1(k) - C*Xm;
   
    K = Pm*C'*inv(C*Pm*C' + R);
    X = Xm + K*yk;
    P = (eye(2) - K*C)*Pm;
    angKF(k,j) = X(1);
    velKF(k,j) = X(2);
    K1(k,j) = K(1);
    K2(k,j) = K(2);

end 

%Steady-state KF for implementation in Simulink
A = [1 1/fs;0 1];
C = [1 0];
K = [K1(k,j) K2(k,j)]'; %Based on tuning parameters above, e.g., K = [0.3945   49.2149]'
X = [Angle1(1,j) 0]';
for k = 1:length(Angle1(:,j))
    Xm = A*X;
    yk = Angle1(k) - C*Xm;
    X = Xm + K*yk;
    angKF(k,j) = X(1);
    velKF(k,j) = X(2);
end
end 

 figure
for j = 1:7
    subplot(4,2,j),plot(t(1:1849),velKF(1:1849,j),t(1:1849),Vel1(:,j)) 
    title(['Velocity : Joint',num2str(j)])
end