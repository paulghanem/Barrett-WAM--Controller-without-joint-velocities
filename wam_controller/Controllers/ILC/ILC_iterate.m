close all 

t1=x_2.time;
t2=q.time;
z=t1(1);
z1=find(t2==z);

x_2=x_2.signals.values;
x_2=transpose(x_2);
x2_KF=x2_KF.signals.values;
x2_KF=transpose(x2_KF);
q=q.signals.values;
q=transpose(q);



 C= [zeros(7,7),eye(7)];

F1 =  diag([0.20519,0.094428,0.094428,0.03,0.001,0.001,0.0003]);

Ts = 0.002;
 tf=4;
t_d = 0:Ts:tf;
yd1= 0.5*(exp(0.5*t_d))-0.5;
yd2= -(0.5*(exp(0.5*t_d))-0.5);
yd3=0*ones(1,tf/Ts+1);
yd4= 1-exp(0.5*t_d);

x2=x2.signals.values; 
x2=transpose(x2);

n = length(yd2);
yd=[yd1;yd2;yd3;yd4;zeros(3,n)];

for j = 1:7
    subplot(4,2,j),plot(t_d,yd(j,:),t_d,x2(j,1001:1000+length(t_d))) 
    title(['Velocity : Joint',num2str(j)])
    
end

figure

for j=1:7
    subplot(4,2,j),plot(t_d,yd(j,:),t_d,x2_KF(j,1001:1000+length(t_d)))
    title(['Velocity_KF : Joint',num2str(j)])
end 


for tk=1:n+1
    if tk==1 
        x_22(:,tk)=zeros(7,1);
    else 
    x_22(:,tk-1)=x_2(:,tk+(2/Ts));
    end 
    B=[zeros(7,7);inv(D(q(:,tk+(2/Ts)))+F1)];
    K(:,:,tk)=(4/(5*Ts))*inv(C*B);
     
end 



e=yd-x_22;
ek1 = e(:,2:n);
 ek1(:,n) = e(:,n);
 u=transpose(u) ;
 u=u(2:8,:);
 
   for i=1:n 
     u(:,i+(2/Ts))=  u(:,i+(2/Ts)) + K(:,:,i)*(ek1(:,i) - e(:,i));
   end
   
        
    u=[transpose(t),transpose(u)];


