clear all 
load('wam7_init.mat');
Ts=0.002;
tf=7;
t=0:Ts:tf;
u=zeros(7,length(t));
u=[transpose(t),transpose(u)];