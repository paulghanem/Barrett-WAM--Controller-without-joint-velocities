clear all 
Ts = 0.00001;
t_d=0:Ts:1;
n1=length(t_d);

    
    posd= [0.6*t_d + exp(-0.6*t_d)-1.1106;
         0.6*t_d + exp(-0.6*t_d)- 2.9490;
         0.5*t_d + exp(-0.5*t_d)-1-0.192;
        -0.5*t_d - exp(-0.5*t_d)+4.1840;
        -0.5*t_d - exp(-0.5*t_d)+(1+0.2825);
        -0.5*t_d - exp(-0.5*t_d)+(1+0.0969);
        -0.5*t_d - exp(-0.5*t_d)+(1-0.2798)];
    
     veld= [0.6-0.6*(exp(-0.6*t_d));
        0.6-0.6*(exp(-0.6*t_d));
       0.5-0.5*(exp(-0.5*t_d)) ;
        -0.5+0.5*exp(-0.5*t_d);
        -0.5+0.5*exp(-0.5*t_d);
        -0.5+0.5*exp(-0.5*t_d);
        -0.5+0.5*exp(-0.5*t_d)];
    
    accd=[0.36*(exp(-0.6*t_d));
          0.36*(exp(-0.6*t_d));
          0.25*(exp(-0.5*t_d)) ;
         -0.25*exp(-0.5*t_d);
         -0.25*exp(-0.5*t_d);
         -0.25*exp(-0.5*t_d);
         -0.25*exp(-0.5*t_d)];
     for tk = 1:n1-1
         
     
     Tau(:,tk)=D(posd(:,tk))*accd(:,tk)+transpose(G(posd(:,tk)));
     end 
     
     
     for j = 1:7
    subplot(4,2,j),plot(t_d(1:length(Tau)),Tau(j,:))
    title(['Torque: Joint',num2str(j)])
     end 
     
     x1(:,1)=[-0.1106,-1.949,-0.192,3.184,0.2825,0.0969,-0.27980]';
     x2(:,1)=zeros(7,1);
     for tk = 1:n1-1
   
      x1(:,tk+1) =  x2(:,tk)*Ts + x1(:,tk);
      x2(:,tk+1) = (-inv(D(x1(:,tk)))*(transpose(G(x1(:,tk))))+ inv(D(x1(:,tk)))*Tau(:,tk))*Ts + x2(:,tk);
     end 
     figure 
     for j = 1:7
    subplot(4,2,j),plot(t_d,x2(j,1:n1))
    title(['Position: Joint',num2str(j)])
    legend('generated trajectory')
     end 
     
     figure 
     for j = 1:7
    subplot(4,2,j),plot(t_d,veld(j,1:n1))
    title(['Position: Joint',num2str(j)])
    legend('desired trajectory')
     end 
     