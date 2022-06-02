function y= rlc_circ
R=1;
C=1;
L=1;
A=[0,1/C;
    -1/L, -R\L];
B=[0;-1/L];
V=1;
Cd=[0,1];
x(:,1)=zeros(2,1);
Ts=0.0001;

for i= 1:200000
    xd(:,i)= A*x(:,i)+B*sin(2*pi*i*Ts);
x(:,i+1)=xd(:,i)*Ts+x(:,i);
 y(:,i)=Cd*x(:,i);   
end 

    







