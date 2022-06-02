clear all
count=1;

Ts = 0.002;

t_d = 0:Ts:2;
n=length(t_d);
 posd = [0.6*t_d + exp(-0.6*t_d)-1.1106;
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
    qdd= [-0.36*(exp(-0.6*t_d));
        0.36*(exp(-0.6*t_d));
       0.25*(exp(-0.5*t_d)) ;
        -0.25*exp(-0.5*t_d);
        -0.25*exp(-0.5*t_d);
        -0.25*exp(-0.5*t_d);
        -0.25*exp(-0.5*t_d)];
    
yd=posd;
t = 0:n-1;
t = Ts*t;

%initial cond
x1 = [[-0.11060;-1.932;-0.19200;3.1;0.2825;0.0969;-0.2798],zeros(7,n-1)];
x2=zeros(7,n);

ek1 = zeros(7,n);
u=zeros(7,n);
C1= [eye(7),zeros(7,7)];

cont=1;

noise_amp = 0;
for k = 1:5
    %initial cond
    x2=[zeros(7,n)];
    x1 = [[-0.11060;-1.932;-0.19200;3.1;0.2825;0.0969;-0.2798],x1(:,2:n)];
    

     for tk = 1:n
        
       x1(:,tk+1) =  x2(:,tk)*Ts + x1(:,tk);
            
 x2(:,tk+1) = ((-inv(D(x1(:,tk))+diag([0.20519,0.094428,0.094428,0.03,0.001,0.001,0.0003])))*(transpose(G(x1(:,tk)))+bq(x2(:,tk)))+ inv(D(x1(:,tk))+diag([0.20519,0.094428,0.094428,0.03,0.001,0.001,0.0003]))*(u(:,tk)+transpose(G(x1(:,tk)))))*Ts + x2(:,tk);
            %isolating joints 
       %x1(5:7,tk+1)=[0.2825;0.0969;-0.2798];
      
         
      x3(:,tk)=(-inv(D(x1(:,tk)))*(transpose(G(x1(:,tk))))+ inv(D(x1(:,tk)))*u(:,tk));
     
    
       
        
       
        
        Ac1=[zeros(7,7), eye(7);
            -inv(D(x1(:,tk)))*L(x1(:,tk),x3(:,tk)) , zeros(7,7)];
    
   Bc=[zeros(7,7);inv(D(x1(:,tk)))];
   
       sysc=ss(Ac1,Bc,C1,0);
       sys=c2d(sysc,Ts);
       [A,B,C,D1]=ssdata(sys);
       
        K(:,:,tk) =inv(C*B);
      
        
    end 

    
   
    
   
    
    e = yd - x1(:,1:n);
   
    ek1 = e(:,2:n);
    ek1(:,n) = e(:,n);
    
   for i=1:n
        
            
        
            u(:,i)=  u(:,i) + K(:,:,i)*(ek1(:,i) - e(:,i));
           
            
            
        end
  

    
    ek2(:,:,count)=ek1;
    
    count = count + 1;
    maxabse1(count) = max(abs(ek1(1,:)));
    maxabse2(count) = max(abs(ek1(2,:)));
    maxabse3(count) = max(abs(ek1(3,:)));
    maxabse4(count) = max(abs(ek1(4,:)));
    maxabse5(count) = max(abs(ek1(5,:)));
    maxabse6(count) = max(abs(ek1(6,:)));
    maxabse7(count) = max(abs(ek1(7,:)));
    
    
    
    meanabse1(count) = mean(abs(ek1(1,:)));
    meanabse2(count) = mean(abs(ek1(2,:)));
    meanabse3(count) = mean(abs(ek1(3,:)));
    meanabse4(count) = mean(abs(ek1(4,:)));
    meanabse5(count) = mean(abs(ek1(5,:)));
    meanabse6(count) = mean(abs(ek1(6,:)));
    meanabse7(count) = mean(abs(ek1(7,:)));
    maxu1(count) = max(u(1,:));
    maxu2(count) = max(u(2,:));
    maxu3(count) = max(u(3,:));
    maxu4(count) = max(u(4,:));
    maxu5(count) = max(u(5,:));
    maxu6(count) = max(u(6,:));
    maxu7(count) = max(u(7,:));
    
    %clear e t t_d t_d1 u x y yd yd1 yd2 yd3
    %K = K/k;
end


%plots
 
 
 
% maxu1
% maxu2
% maxu3
% maxu4
% maxu5
% maxu6
% maxu7
 
figure
subplot(2,4,1),plot(maxabse1),grid,ylabel('max(|Error1|)')
subplot(2,4,5),plot(meanabse1),grid,ylabel('average(|Error1|)')
xlabel('Number of Iterations')
 
subplot(2,4,2),plot(maxabse2),grid,ylabel('max(|Error2|)')
subplot(2,4,6),plot(meanabse2),grid,ylabel('average(|Error2|)')
xlabel('Number of Iterations')
 
subplot(2,4,3),plot(maxabse3),grid,ylabel('max(|Error3|)')
subplot(2,4,7),plot(meanabse3),grid,ylabel('average(|Error3|)')
xlabel('Number of Iterations')
 
subplot(2,4,4),plot(maxabse4),grid,ylabel('max(|Error4|)')
subplot(2,4,8),plot(meanabse4),grid,ylabel('average(|Error4|)')
xlabel('Number of Iterations')
figure
subplot(2,3,1),plot(maxabse5),grid,ylabel('max(|Error5|)')
subplot(2,3,4),plot(meanabse5),grid,ylabel('average(|Error5|)')
xlabel('Number of Iterations')
 
subplot(2,3,2),plot(maxabse6),grid,ylabel('max(|Error6|)')
subplot(2,3,5),plot(meanabse6),grid,ylabel('average(|Error6|)')
xlabel('Number of Iterations')
 
subplot(2,3,3),plot(maxabse7),grid,ylabel('max(|Error7|)')
subplot(2,3,6),plot(meanabse7),grid,ylabel('average(|Error7|)')
xlabel('Number of Iterations')
 
 
 
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,yd(j,:),t_d,x1(j,:)) 
    title(['Position Error: Joint',num2str(j)])
end