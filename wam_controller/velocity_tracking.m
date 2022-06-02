clc 
clear all
count=0;

Ts = 0.002;

t_d = 0:Ts:2;


yd= [0.5-0.5*(exp(-0.5*t_d));
0.8-0.8*(exp(-0.8*t_d));
0*ones(1,length(t_d));
 -0.7+0.7*exp(-0.7*t_d)];

n = length(yd);
yd=[yd;zeros(3,n)];

x= [0.5*t_d + exp(-0.5*t_d)-1.1106;
    0.8*t_d + exp(-0.8*t_d)- 2.9490;
    -0.192*ones(1,length(t_d));
   -0.7*t_d - exp(-0.7*t_d)+4.1840];

n = length(t_d);

x=[x;0.2825*ones(1,n);0.0969*ones(1,n);-0.2798*ones(1,n)];

t = 0:n-1;
t = Ts*t;

%initial cond
 x1 = [[-0.11060;-1.9490;-0.19200;3.1840;0.28250;0.09690;-0.27980]*ones(1,n-1)];
 x2=zeros(7,n);
u=zeros(7,n);

ek1 = zeros(7,n);

C= [zeros(7,7),eye(7)];

cont=1;

noise_amp = 0;
for k = 1:10
    %initial cond
     x1 = [[-0.11060;-1.9490;-0.19200;3.1840;0.28250;0.09690;-0.27980]*ones(1,n-1)];
 x2=zeros(7,n);
    

    for tk = 1:n
      x1(:,tk+1) =  x2(:,tk)*Ts + x1(:,tk);
            x1(:,tk)=x1(:,tk)+0.001*(0.5-rand(7,1));
 x2(:,tk+1) = (-inv(D(x1(:,tk)))*(transpose(G(x1(:,tk))))+ inv(D(x1(:,tk)))*u(:,tk))*Ts + x2(:,tk);
       %isolating joints 
       %x1(5:7,tk+1)=[0.2825;0.0969;-0.2798];
   %differentiator   
       x_2(:,tk)=(x1(:,tk+1)-x1(:,tk))/Ts;
         B= [zeros(7,7);inv(D(x1(:,tk)))];
         K(:,:,tk)=inv(C*B);
         
        
       
    end 
    B= [zeros(7,7);inv(D(x1(:,n)))];
         K(:,:,n)=inv(C*B);
         
         
    

    
   
    
   
    
    e = yd - x2(:,1:n);
   
    ek1 = e(:,2:n);
    ek1(:,n) = e(:,n);
   
    
   for i=1:n
        
            
        
            u(:,i)=  u(:,i) + ((ek1(:,i).^2)/(2+ek1(:,i).^2))*K(:,:,i)*(ek1(:,i) - e(:,i));
            
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


%plots
figure
plot(t_d,x2(1,1:n),t_d,x_2(1,1:n))

figure
plot(t_d,yd(1,:),t_d,x2(1,1:n))


xlabel('time')
ylabel('position')

legend('desired signal','actual signal')

title('joint1 simulation ')

figure
plot(t_d,yd(2,:),t_d,x2(2,1:n))


xlabel('time')
ylabel('position')

legend('desired signal','actual signal')

title('joint2 simulation ')
figure
plot(t_d,yd(3,:),t_d,x2(3,1:n))


xlabel('time')
ylabel('position')

legend('desired signal','actual signal')

title('joint3 simulation ')

figure
plot(t_d,yd(4,:),t_d,x2(4,1:n))


xlabel('time')
ylabel('position')

legend('desired signal','actual signal')

title('joint4 simulation ')
figure
plot(t_d,yd(5,:),t_d,x2(5,1:n))


xlabel('time')
ylabel('position')

legend('desired signal','actual signal')

title('joint5 simulation ')
figure

plot(t_d,yd(6,:),t_d,x2(6,1:n))


xlabel('time')
ylabel('position')

legend('desired signal','actual signal')

title('joint6 simulation ')
figure
plot(t_d,yd(7,:),t_d,x2(7,1:n))


xlabel('time')
ylabel('position')

legend('desired signal','actual signal')

title('joint7 simulation ')

figure
subplot(1,4,1),plot(t_d,yd(1,:),t_d,x2(1,1:n))
grid,xlabel('time'),ylabel('position of joint1 2nd iteration K= 1e-3*(1/Ts)*inv(C*B)')
legend('desired velocity','actual velocity')

subplot(1,4,2),plot(t_d,yd(2,:),t_d,x2(2,1:n))
grid,xlabel('time'),ylabel('position  of joint2 2nd iteration  K= 1e-3*(1/Ts)*inv(C*B) ')
legend('desired velocity','actual velocity')

subplot(1,4,3),plot(t_d,yd(3,:),t_d,x2(3,1:n))
grid,xlabel('time'),ylabel('position of joint3 2nd iteration K=1e-3*(1/Ts)*inv(C*B) ')
legend('desired velocity','actual velocity')

subplot(1,4,4),plot(t_d,yd(4,:),t_d,x2(4,1:n))
grid,xlabel('time'),ylabel('position of joint 2nd iteration K= 1e-3*(1/Ts)*inv(C*B) ')
legend('desired velocity','actual velocity')
maxu1
maxu2
maxu3
maxu4
maxu5
maxu6
maxu7

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
