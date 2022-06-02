dfunction x = DC_motor;
J = 0.02;
b = 0.001;
K = 0.1;
R = 4;
L = 1000;

A = [0 1 0
    0 -b/J K/J
    0 -K/L -R/L];
B = [0 ; 0 ; 1/L];
C = [1  0  0];
D = 0;

x(:,1)= zeros(3,1);
Ts=0.0000001;

for i =1:15 
    
xd(:,i)= A*x(:,i)+B*0.001*sin(2*pi*i*Ts);
x(:,i+1)=xd(:,i)*Ts+x(:,i)

end 
 


