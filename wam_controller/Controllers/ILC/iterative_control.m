clc 
clear 

 
 
R = 0.6;
Km = 0.075;
L = 0.0024;
J = 2.1146e-004;
Kf = 0.0011;
 
A = [-R/L -Km/L; Km/J -Kf/J];
B = [1/L; 0];
C = [1  0];
 
 
count = 0;
Ts = 1e-2;
 
sys = ss(A, B, C, 0);
sys = c2d(sys,Ts);
A = sys.A;
B = sys.B;
C = [1 0];
 
 
 
t_d = 0:Ts:0.5;
t_d1 = 0:Ts:2.25;
yd1 = 40*sin(2*pi*t_d - pi/2) + 40;
yd2 = 30*sin(2*pi*2*t_d1 - 3*pi/2) + 50;
yd3 = 20*ones(1,floor(5/Ts));
 
yd = [ yd1 yd2 yd3];
 
% Step
% t_d = 0:Ts:0.25;
% yd1 = 40*sin(2*pi*2*t_d - pi/2) + 40;
% yd2 = 80*ones(1,floor(6/Ts));
%
% yd = [ yd1 yd2 ];
 
n = length(yd);
 
t = 0:n-1;
t = Ts*t;
 
x1 = zeros(1,n);
x2 = x1;
y = zeros(1,n);
e = zeros(1,n);
u = zeros(1,n);
u1 = u;
K1 = 0.1/(C*B)
K = K1;
noise_amp = 0;
for k = 1:5
   
    for tk = 1:n-1
        x1(tk+1) = A(1,1)*x1(tk) + A(1,2)*x2(tk) + B(1)*u(tk);
        x2(tk+1) = A(2,1)*x1(tk) + A(2,2)*x2(tk) + B(2)*u(tk);
    end
   
    e = yd - x2;
    ek1 = e(2:n);
    ek1(n) = e(n);
   
    u = u + K*ek1;
   
    count = count + 1;
    maxabse(count) = max(abs(e));
    meanabse(count) = mean(abs(e));
    maxu(count) = max(u);
    %clear e t t_d t_d1 u x y yd yd1 yd2 yd3
    %K = K/k;
end
 
plot(t,x2,'g',t,yd,'r')
title('Track')
figure, plot(t,abs(e))
title('Error')
figure, plot(t,u)
title('input')
 
 
maxu
figure
subplot(2,1,1),plot(maxabse),grid,ylabel('max(|Error|)')
subplot(2,1,2),plot(meanabse),grid,ylabel('average(|Error|)')
xlabel('Number of Iterations')
ratio_meane = max(meanabse)/min(meanabse)
ratio_maxe = max(maxabse)/min(maxabse)