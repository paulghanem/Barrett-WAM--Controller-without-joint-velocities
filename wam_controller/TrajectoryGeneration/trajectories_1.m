function yd=trajectories_1(traj,wt,a,amp,Ts,Tf)

t_d = 0:Ts:Tf-Ts;
n1 = length(t_d);


if traj==1
  
     posd = a*t_d + amp*exp(-a*t_d)-1;
         
    
     veld= a-a*amp*(exp(-a*t_d));
         
    
    accd= -(a^2)*amp*(exp(-a*t_d));
        
elseif traj==2
    posd = 1+amp*sin((wt)*t_d-pi/2);
  

veld= amp*(wt)*cos((wt)*t_d-pi/2);
    

accd = -amp*((wt)^2)*sin((wt)*t_d-pi/2);
    
    end 



yd=[posd;veld;accd];



end 
