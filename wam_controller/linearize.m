clear all
clc
Ts=1/500;
u=zeros(7,1);
x1=u;
x2=x1;
x_1=ones(7,1);
x_2=x_1;
u=zeros(14,1);

x=[x1;x2];


[A B C D ]= dlinmod('plant_model',Ts,x,u)
