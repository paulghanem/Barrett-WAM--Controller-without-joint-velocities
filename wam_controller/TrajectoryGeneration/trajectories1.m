function yd=trajectories1(traj,wt,a,Ts,Tf)

t_d = 0:Ts:Tf-Ts;
n1 = length(t_d);


if traj==0
    
posd = [-0.1106+1+sin((wt)*t_d-pi/2);
    -1.932+1+sin((wt)*t_d-pi/2);
    -0.1920+0.5+0.5*sin((wt)*t_d-pi/2);
    2.1+sin((wt)*t_d+pi/2)];

veld= [(wt)*cos((wt)*t_d-pi/2);
    (wt)*cos((wt)*t_d-pi/2);
    0.5*(wt)*cos((wt)*t_d-pi/2);
    (wt)*cos((wt)*t_d+pi/2)];



accd = [-((wt)^2)*sin((wt)*t_d-pi/2);
    -((wt)^2)*sin((wt)*t_d-pi/2);
    -0.5*((wt)^2)*sin((wt)*t_d-pi/2);
    -((wt)^2)*sin((wt)*t_d+pi/2)];
elseif traj==2
   b=a;
     posd = [a*t_d + exp(-a*t_d)-1.1106;
         a*t_d + exp(-a*t_d)- 2.9490;
         a*t_d + exp(-a*t_d)-1-0.0192;
        -a*t_d - exp(-a*t_d)+4.1840];
        
     veld= [a-a*(exp(-a*t_d));
         a-a*(exp(-a*t_d));
         a-a*(exp(-a*t_d)) ;
        -a+a*exp(-a*t_d)];
    
    accd= [-(a^2)*(exp(-a*t_d));
        (a^2)*(exp(-a*t_d));
       (a^2)*(exp(-a*t_d)) ;
        -(a^2)*exp(-a*t_d)];
elseif traj==1
    posd = [-0.1106+1+sin((wt)*t_d-pi/2);
    -1.949+1+sin((wt)*t_d-pi/2);
    -0.01920+0.5+0.5*sin((wt)*t_d-pi/2);
    2.184+sin((wt)*t_d+pi/2)];
  

veld= [(wt)*cos((wt)*t_d-pi/2);
    (wt)*cos((wt)*t_d-pi/2);
    0.5*(wt)*cos((wt)*t_d-pi/2);
    (wt)*cos((wt)*t_d+pi/2)];



accd = [-((wt)^2)*sin((wt)*t_d-pi/2);
    -((wt)^2)*sin((wt)*t_d-pi/2);
    -0.5*((wt)^2)*sin((wt)*t_d-pi/2);
    -((wt)^2)*sin((wt)*t_d+pi/2)];
    end 



yd=[posd;veld;accd];



end 
