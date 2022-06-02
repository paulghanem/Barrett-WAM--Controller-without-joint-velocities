function velKF = KF(Angle1) 

Angle1=transpose(Angle1);

fs = 500; % Sample rate
Vel1 = diff(Angle1)*fs; % Estimate by direct differencing
t = [0:length(Vel1)]/fs;
Ac = [0 1;0 0];
Bc = [0 1]';
C = [1 0];
sysc = ss(Ac, Bc, C, 0);
sys = c2d(sysc,1/fs);
[A,B,C,D] = ssdata(sys);
X = [Angle1(1) 0]'; % Initial condition (guess)
%Tuning parameters: For example for smoother performance you can decrease the value of Q at the cost of slow response or larger transient response errors
Q = 1000;
R = 1e-3;
P = [1e-4 0;0 1];
% KF loop
for k = 1:length(Angle1)
    Xm = A*X;
    Pm = A*P*A' + B*Q*B';
    yk = Angle1(k) - C*Xm;
   
    K = Pm*C'*inv(C*Pm*C' + R);
    X = Xm + K*yk;
    P = (eye(2) - K*C)*Pm;
    angKF(k) = X(1);
    velKF(k) = X(2);
    K1(k) = K(1);
    K2(k) = K(2);
end

%Steady-state KF for implementation in Simulink
A = [1 1/fs;0 1];
C = [1 0];
K = [K1(k) K2(k)]'; %Based on tuning parameters above, e.g., K = [0.3945   49.2149]'
X = [Angle1(1) 0]';
for k = 1:length(Angle1)
    Xm = A*X;
    yk = Angle1(k) - C*Xm;
    X = Xm + K*yk;
    angKF(k) = X(1);
    velKF(k) = X(2);
end

 






end 



