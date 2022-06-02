function x= robot_iterative_control1

count = 0;
Ts = 0.002;

t_d = 0:Ts:0.5;
t_d1 = 0:Ts:2.25;
yd1 = 0.5*sin(2*pi*t_d) ;
yd2 = 0.5*sin(2*pi*2*t_d1);
yd3 = zeros(1,floor(1/Ts));
 
yd = [ yd1 yd2 yd3];

n = length(yd);
yd= ones(7,1)*yd;

 
t = 0:n-1;
t = Ts*t;

 
x1 = zeros(7,n);
x2 = x1;
y = zeros(7,n);
ek1 = zeros(7,n);
u = zeros(7,n);
u1 = u;
C= [zeros(7,7),eye(7)];


noise_amp = 0;
for k = 1:5
   x1 = zeros(7,n);
x2 = x1;
    for tk = 1:n-1
        x1(:,tk+1) =  x2(:,tk); 
        
        x2(:,tk+1) = -inv(D(x1(:,tk))+diag([0.20519,0.094428,0.094428,0.03,0.001,0.001,0.0003]))*(transpose(G(x1(:,tk)))+bq(x2(:,tk)))+ inv(D(x1(:,tk))+diag([0.20519,0.094428,0.094428,0.03,0.001,0.001,0.0003]))*u(:,tk);
   
   B(:,:,tk)= [zeros(7,7);inv(D(x1(:,tk))+diag([0.20519,0.094428,0.094428,0.03,0.001,0.001,0.0003]))];
   B(:,:,n)= [zeros(7,7);inv(D(x1(:,n))+diag([0.20519,0.094428,0.094428,0.03,0.001,0.001,0.0003]))];
   K(:,:,tk) = inv(C*B(:,:,tk));
   K(:,:,n)=  inv(C*B(:,:,n));
   K(1,:,tk)= 0.06*K(1,:,tk);
   K(2,:,tk)= 0.1*K(2,:,tk);
   K(3,:,tk)= 0.1*K(3,:,tk);
   K(4,:,tk)= 0.1*K(4,:,tk);
   K(5,:,tk)= 0.1*K(5,:,tk);
   K(6,:,tk)= 0.1*K(6,:,tk);
   K(7,:,tk)= 0.1*K(7,:,tk);
   
%joint limits    
 if x1(1,tk+1)<-2.6
   x2(1,tk+1)=0;
   x1(1,tk+1) = -2.6;
end 
if x1(1,tk+1)>2.6
   x2(1,tk+1)=0;
    x1(1,tk+1) = 2.6;
end 
if x1(2,tk+1)<-2
   x2(2,tk+1)=0;
    x1(2,tk+1) = -2;
end 
if x1(2,tk+1)>2
   x2(2,tk+1)=0;
    x1(2,tk+1) = 2;
end 

if x1(3,tk+1)<-2.8
    x1(3,tk+1) = -2.8;
    x2(3,tk+1)=0;
end 
if x1(3,tk+1)>2.8
    x1(3,tk+1) = 2.8;
    x2(3,tk+1)=0;
end 

if x1(4,tk+1)>3.1
    x1(4,tk+1) = 3.1;
    x2(4,tk+1)=0;
end 
if x1(4,tk+1)<-0.9
    x1(4,tk+1) = -0.9;
    x2(4,tk+1)=0;
    
end 

if x1(5,tk+1)<-4.8
    x1(5,tk+1) = -4.8;
    x2(5,tk+1)=0;
end 
if x1(5,tk+1)>1.3
    x1(5,tk+1) = 1.3;
    x2(5,tk+1)=0;
end 

if x1(6,tk+1)<-1.6
    x1(6,tk+1) = -1.6;
    x2(6,tk+1)=0;
end 

if x1(6,tk+1)>1.6
    x1(6,tk+1) = 1.6;
    x2(6,tk+1)=0;
end 

if x1(7,tk+1)>2.2
    x1(7,tk+1) = 2.2;
    x2(7,tk+1)=0;
end 
if x1(7,tk+1)<-2.2
    x1(7,tk+1) = -2.2;
    x2(7,tk+1)=0;
end 
   
    end
     e = yd - x2;
    ek1 = e(:,2:n);
    ek1(:,n) = e(:,n);
    for i=1:n
    
    u(:,i) = u(:,i) + K(:,:,i)*ek1(:,i);
    end 
     
   
   
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
plot(t,x2(1,:),'g',t,yd(1,:),'r')
title('Track1')
plot(t,x2(2,:),'g',t,yd(2,:),'r')
title('Track2')
plot(t,x2(3,:),'g',t,yd(3,:),'r')
title('Track3')
plot(t,x2(4,:),'g',t,yd(4,:),'r')
title('Track4')
plot(t,x2(5,:),'g',t,yd(5,:),'r')
title('Track5')
plot(t,x2(6,:),'g',t,yd(6,:),'r')
title('Track6')
plot(t,x2(7,:),'g',t,yd(7,:),'r')
title('Track7')


figure, plot(t,abs(ek1(1,:)))
title('Error1')
figure, plot(t,abs(ek1(2,:)))
title('Error2')
figure, plot(t,abs(ek1(3,:)))
title('Error3')
figure, plot(t,abs(ek1(4,:)))
title('Error4')
figure, plot(t,abs(ek1(5,:)))
title('Error5')
figure, plot(t,abs(ek1(6,:)))
title('Error6')
figure, plot(t,abs(ek1(7,:)))
title('Error7')



figure, plot(t,u(1,:))
title('input1')
figure, plot(t,u(2,:))
title('input2')
figure, plot(t,u(3,:))
title('input3')
figure, plot(t,u(4,:))
title('input4')
figure, plot(t,u(5,:))
title('input5')
figure, plot(t,u(6,:))
title('input6')
figure, plot(t,u(7,:))
title('input7')

maxu1
maxu2
maxu3
maxu4
maxu5
maxu6
maxu7

figure
subplot(2,1,1),plot(maxabse1),grid,ylabel('max(|Error1|)')
subplot(2,1,2),plot(meanabse1),grid,ylabel('average(|Error1|)')
xlabel('Number of Iterations')
figure
subplot(2,1,1),plot(maxabse2),grid,ylabel('max(|Error2|)')
subplot(2,1,2),plot(meanabse2),grid,ylabel('average(|Error2|)')
xlabel('Number of Iterations')
figure
subplot(2,1,1),plot(maxabse3),grid,ylabel('max(|Error3|)')
subplot(2,1,2),plot(meanabse3),grid,ylabel('average(|Error3|)')
xlabel('Number of Iterations')
figure
subplot(2,1,1),plot(maxabse4),grid,ylabel('max(|Error4|)')
subplot(2,1,2),plot(meanabse4),grid,ylabel('average(|Error4|)')
xlabel('Number of Iterations')
figure
subplot(2,1,1),plot(maxabse5),grid,ylabel('max(|Error5|)')
subplot(2,1,2),plot(meanabse5),grid,ylabel('average(|Error5|)')
xlabel('Number of Iterations')
figure
subplot(2,1,1),plot(maxabse6),grid,ylabel('max(|Error6|)')
subplot(2,1,2),plot(meanabse6),grid,ylabel('average(|Error6|)')
xlabel('Number of Iterations')
figure
subplot(2,1,1),plot(maxabse7),grid,ylabel('max(|Error7|)')
subplot(2,1,2),plot(meanabse7),grid,ylabel('average(|Error7|)')
xlabel('Number of Iterations')


ratio_meane1 = max(meanabse1)/min(meanabse1)
ratio_maxe1 = max(maxabse1)/min(maxabse1)
ratio_meane2 = max(meanabse2)/min(meanabse2)
ratio_maxe2 = max(maxabse2)/min(maxabse2)
ratio_meane3 = max(meanabse3)/min(meanabse3)
ratio_maxe3 = max(maxabse3)/min(maxabse3)
ratio_meane4 = max(meanabse4)/min(meanabse4)
ratio_maxe4 = max(maxabse4)/min(maxabse4)
ratio_meane5 = max(meanabse5)/min(meanabse5)
ratio_maxe5 = max(maxabse5)/min(maxabse5)
ratio_meane6 = max(meanabse6)/min(meanabse6)
ratio_maxe6 = max(maxabse6)/min(maxabse6)
ratio_meane7 = max(meanabse7)/min(meanabse7)
ratio_maxe7 = max(maxabse7)/min(maxabse7)

K(1,:,tk)= 0.005*K(1,:,tk);
   K(2,:,tk)= 0.005*K(2,:,tk);
   K(3,:,tk)= 0.005*K(3,:,tk);
   K(4,:,tk)= 0.005*K(4,:,tk);
   K(5,:,tk)= 0.005*K(5,:,tk);
   K(6,:,tk)= 0.005*K(6,:,tk);
   K(7,:,tk)= 0.005*K(7,:,tk);
   K(1,:,n)= 0.005*K(1,:,tk);
   K(2,:,n)= 0.005*K(2,:,tk);
   K(3,:,n)= 0.005*K(3,:,tk);
   K(4,:,n)= 0.005*K(4,:,tk);
   K(5,:,n)= 0.005*K(5,:,tk);
   K(6,:,n)= 0.005*K(6,:,tk);
   K(7,:,n)= 0.005*K(7,:,tk);


end 
  
