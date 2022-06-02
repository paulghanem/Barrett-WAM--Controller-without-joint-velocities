u1=u;
count=1;

Ts = 0.002;

t_d = 0:Ts:6;
yd= [-0.1106+sin((pi/4)*t_d);
    -0.932+sin((pi/4)*t_d-pi/2);
    -0.1920+0.5*sin((pi/4)*t_d);
    2.1+sin((pi/4)*t_d+pi/2)];

n = length(yd);

yd=[yd;0.2825*ones(1,n);0.0969*ones(1,n);-0.2798*ones(1,n)];

t = 0:n-1;
t = Ts*t;

%initial cond
x1 = [[-0.11060;-2;-0.19200;3.1;0.2825;0.0969;-0.2798],zeros(7,n-1)];
x2=zeros(7,n);

ek1 = zeros(7,n);

C= [eye(7),zeros(7,7)];

cont=1;

noise_amp = 0;
for k = 1:5
    %initial cond
    x2=[zeros(7,1),x2(:,2:n)];
    x1 = [[-0.11060;-2;-0.19200;3.1;0.2825;0.0969;-0.2798],x1(:,2:n)];
    

    for tk = 1:n
       x1(:,tk+1) =  x2(:,tk)*Ts + x1(:,tk);
            
 x2(:,tk+1) = (-inv(D(x1(:,tk)))*(transpose(G(x1(:,tk))))+ inv(D(x1(:,tk))+diag([0.20519,0.094428,0.094428,0.03,0.001,0.001,0.0003]))*u(:,tk))*Ts + x2(:,tk);
       %isolating joints 
       %x1(5:7,tk+1)=[0.2825;0.0969;-0.2798];
      
         yd(5:7,tk)=x1(5:7,tk);
       F(:,tk)=-inv(D(x1(:,tk)))*transpose(G(x1(:,tk)));
    end 
     F(:,n+1)=-inv(D(x1(:,n+1)))*transpose(G(x1(:,n+1)));
    for tk= 1:n 
       
        
        Ac=[(F(:,tk+1)-F(:,tk))/(x1(1,tk+1)-x1(1,tk)),(F(:,tk+1)-F(:,tk))/(x1(2,tk+1)-x1(2,tk)),(F(:,tk+1)-F(:,tk))/(x1(3,tk+1)-x1(3,tk)),(F(:,tk+1)-F(:,tk))/(x1(4,tk+1)-x1(4,tk)),(F(:,tk+1)-F(:,tk))/(x1(5,tk+1)-x1(5,tk)),(F(:,tk+1)-F(:,tk))/(x1(6,tk+1)-x1(6,tk)),(F(:,tk+1)-F(:,tk))/(x1(7,tk+1)-x1(7,tk))];
 
        for i=1:7
        if (x1(i,tk+1)-x1(i,tk))==0
            E=-inv(D(x1(:,tk)));
            Ac(i,:)= E(i,:)*u(:,tk)/x1(:,tk);
        end 
        end 
        
        Ac=[zeros(7,7), eye(7);
            Ac , zeros(7,7)];
    
   Bc=[zeros(7,7);inv(D(x1(:,tk)))];
   
       sysc=ss(Ac,Bc,C,0);
       sys=c2d(sysc,Ts);
       [A,B,C,D1]=ssdata(sys);
       
        K(:,:,tk) =2e-20*inv(C*B);
        K(5:7,:,tk)=zeros(3,7);
        K(:,5:7,tk)=zeros(7,3);
        
    end 

    
   
    
   
    
    e = yd - x1(:,1:n);
    e(5:7,:)=zeros(3,n);
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



figure
subplot(1,4,1),plot(t_d,yd(1,:),t_d,x1(1,1:n))
grid,xlabel('time'),ylabel('position of joint1 5th iteration K=2e-19')
legend('desired signal','actual signal')

subplot(1,4,2),plot(t_d,yd(2,:),t_d,x1(2,1:n))
grid,xlabel('time'),ylabel('position  of joint2 5th iteration   K=2e-19')
legend('desired signal','actual signal')

subplot(1,4,3),plot(t_d,yd(3,:),t_d,x1(3,1:n))
grid,xlabel('time'),ylabel('position of joint3 5th iteration   K=2e-19')
legend('desired signal','actual signal')

subplot(1,4,4),plot(t_d,yd(4,:),t_d,x1(4,1:n))
grid,xlabel('time'),ylabel('position of joint 5th iteration   K=2e-19')
legend('desired signal','actual signal')


figure
subplot(1,4,1),plot(t_d,ek2(1,:,1),t_d,ek2(1,:,2),t_d,ek2(1,:,2),t_d,ek2(1,:,2),t_d,ek2(1,:,5))
grid,xlabel('time'),ylabel('error at joint 1 for 5 iterations  K=2e-20*inv(C*B)')
legend('iteration1','iteration 2','iteration 3','iteration 4','iteration 5')

subplot(1,4,2),plot(t_d,ek2(2,:,1),t_d,ek2(2,:,2),t_d,ek2(2,:,2),t_d,ek2(2,:,2),t_d,ek2(2,:,5))
grid,xlabel('time'),ylabel('error at joint 2 for 5 iterations  K=2e-20*inv(C*B)')
legend('iteration1','iteration 2','iteration 3','iteration 4','iteration 5')


subplot(1,4,3),plot(t_d,ek2(3,:,1),t_d,ek2(3,:,2),t_d,ek2(3,:,2),t_d,ek2(3,:,2),t_d,ek2(3,:,5))
grid,xlabel('time'),ylabel('error at joint 3 for 5 iterations K=2e-20*inv(C*B)')
legend('iteration1','iteration 2','iteration 3','iteration 4','iteration 5')


subplot(1,4,4),plot(t_d,ek2(4,:,1),t_d,ek2(4,:,2),t_d,ek2(4,:,3),t_d,ek2(4,:,4),t_d,ek2(4,:,5))
grid,xlabel('time'),ylabel('error at joint 4 for 5 iterations K=2e-20*inv(C*B)')
legend('iteration1','iteration 2','iteration 3','iteration 4','iteration 5')



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
