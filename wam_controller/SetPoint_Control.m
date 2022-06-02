clear all
arm_home = [ -0.1106   -1.9490   -0.1920    3.1840    0.2825    0.0969   -0.2798];
close all
count=0;
n = 14;p = 7; q = 14;
Ts = 1e-3;
t_d = 0:Ts:2e3*Ts;
count=0;
count1=0;
n1=length(t_d);
for k = 3
    a1 = pi/3;
    a2 = 2*pi/2;
     posd(k,:) = a1*(1-cos(a2*t_d))+arm_home(k);
    veld(k,:) = a1*a2*sin(a2*t_d);
    accd(k,:) = a1*a2*a2*cos(a2*t_d);
end
for k = [5 6 7]
    a1 = 2*pi/3;
    a2 = pi/2;
    if k==6 
        a1=pi/2;
    end 
    posd(k,:) = a1*(1-cos(a2*t_d))+arm_home(k);
    veld(k,:) = a1*a2*sin(a2*t_d);
    accd(k,:) = a1*a2*a2*cos(a2*t_d);
    for tk = 1:2/Ts
        if k==5
    if posd(5,tk) > 0.71
    if count1 < 1
        tk1=tk*Ts
        count1=count1+1;
        t_d1=0:Ts:(length(t_d(tk:n1))-1)*Ts;
        posd(k,tk:n1) = a1*(cos(a2*t_d1)-(1-0.71/a1));
    veld(k,tk:n1) = -a1*a2*sin(a2*t_d1);
    accd(k,tk:n1) = -a1*a2*a2*cos(a2*t_d1);
    end 
    end
        end 
    if k==6
    if posd(k,tk) > 1.5
    if count < 1
        tk2=tk*Ts
        count=count+1;
        t_d1=0:Ts:(length(t_d(tk:n1))-1)*Ts;
        posd(k,tk:n1) = a1*(cos(a2*t_d1)-(1-1.5/a1));
    veld(k,tk:n1) = -a1*a2*sin(a2*t_d1);
    accd(k,tk:n1) = -a1*a2*a2*cos(a2*t_d1);
    end 
    end 
   
    
        
    
    end
    end 
end
% for k = 5
%     a1 = pi/8;
%     a2 = 2*pi/25;
%      posd(k,:) = a1*(1-cos(a2*t_d));
%     veld(k,:) = a1*a2*sin(a2*t_d);
%     accd(k,:) = a1*a2*a2*cos(a2*t_d);
% end
% for k = 6
%     a1 = -pi/12;
%     a2 = 2*pi/30;
%      posd(k,:) = a1*(1-cos(a2*t_d));
%     veld(k,:) = a1*a2*sin(a2*t_d);
%     accd(k,:) = a1*a2*a2*cos(a2*t_d);
% end
% for k = 7
%     a1 = pi/4;
%     a2 = 2*pi/10;
%      posd(k,:) = a1*(1-cos(a2*t_d));
%     veld(k,:) = a1*a2*sin(a2*t_d);
%     accd(k,:) = a1*a2*a2*cos(a2*t_d);
% end
for k = [1 2 4]
    a1 = pi/3;
    a2 = pi/2;
    if k==4 
        posd(k,:) = -a1*(1-cos(a2*t_d))+arm_home(k);
    veld(k,:) = -a1*a2*sin(a2*t_d);
    accd(k,:) = -a1*a2*a2*cos(a2*t_d);
    else 
    
     posd(k,:) = a1*(1-cos(a2*t_d))+arm_home(k);
    veld(k,:) = a1*a2*sin(a2*t_d);
    accd(k,:) = a1*a2*a2*cos(a2*t_d);
    end 
end
yd = [posd;veld];
alfa = 1;
yd = alfa*yd;
t = 0:n1-1;
t = Ts*t;
%initial conditions
x1 = alfa*posd(:,1);
x2 = alfa*veld(:,1);
U(:,1) = zeros(7,1);

a=diag(50*ones(7,1));
b=diag(100*ones(7,1));
Kp=diag(8000*ones(7,1));
Kd=diag(5*ones(7,1));

for tk = 1:n1-1
    
    e=x1(:,tk)-posd(:,tk);
    qc=transpose(arm_home);
    %qc=posd(:,tk);
  
    v=qc+b*e;
    U(:,tk)=-Kp*e - Kd*v + D(x1(:,tk))*accd(:,tk)+transpose(G(x1(:,tk)))+bq(veld(:,tk));
  
      
           x1(:,tk+1) =  x2(:,tk)*Ts + x1(:,tk);
        x2(:,tk+1) = (-inv(D(x1(:,tk)))*(transpose(G(x1(:,tk)))+bq(x2(:,tk)))+ inv(D(x1(:,tk)))*U(:,tk))*Ts + x2(:,tk);
       
        
   
    
  
    
  
end
% figure
% subplot(2,1,1),plot(Ek)
% subplot(2,1,2),plot(normP)
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,yd(j+7,:))
    title(['Velocity: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,yd(j+7,:),t_d,x2(j,1:n1))
    title(['Velocity: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d(1:length(U)),U(j,:)),grid
    title(['Torque: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,yd(j,:),t_d,x1(j,1:n1))
    title(['Position: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d,yd(j,:)-x1(j,1:n1))
    title(['Position ERROR: Joint',num2str(j)])
end

 
 
 