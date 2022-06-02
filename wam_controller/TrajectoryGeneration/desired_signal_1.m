clear all
close all
 load('wam7_init.mat')
traj = 1;
Ts = 1/500;
a = 0.5;
wt = pi/2;
Tf = 30;
yd=trajectories(traj,wt,a,Ts,Tf);
yd = 1*yd;
posd = yd(1:7,:);
veld = yd(8:14,:);
accd = yd(15:21,:);
t_d = 0:Ts:Tf-Ts;
n1 = length(t_d);


x1=transpose(posd);
 x1=[ones(2/Ts,1)*x1(1,:);x1];
 t_d1=0:Ts:Tf+2-Ts;
 n2=length(t_d1);
    
    posd=[transpose(t_d1),x1];
     
     
    xd1=transpose(veld);
 xd1=[ones(2/Ts,1)*xd1(1,:);xd1];
 
     
    veld=[transpose(t_d1),xd1];
    
    xdd1=transpose(accd);
 xdd1=[ones(2/Ts,1)*zeros(1,7);xdd1];
 accd=[transpose(t_d1),xdd1];   
 
 
    for tk=1:n2-1
        fposd(tk,:) = x1(tk+1,:);
        fposd(n2,:)= x1(n2,:);
        
        fveld(tk,:)= xd1(tk+1,:);
        fveld(n2,:)= xd1(n2,:);
        
         faccd(tk,:)= xdd1(tk+1,:);
        faccd(n2,:)= xdd1(n2,:);
        
        delposd(tk+1,:)=x1(tk,:);
        delveld(tk+1,:)=xd1(tk,:);
        delposd(1,:)=x1(1,:);
   
    end     
    
    fposd=[transpose(t_d1),fposd];
    fveld=[transpose(t_d1),fveld];
    delposd=[transpose(t_d1),delposd];
    delveld=[transpose(t_d1),delveld];
    faccd=[transpose(t_d1),faccd];
    
  
    