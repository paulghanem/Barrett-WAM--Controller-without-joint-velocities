clear all 
Ts=0.00001 ;
load U

t_d = 0:Ts:1;
t_d1= 0:Ts:1-Ts;
n=length(t_d);
 x1(:,1)=[-0.1106,-1.949,-0.192,3.184,0.2825,0.0969,-0.27980]';
     x2(:,1)=zeros(7,1);



for tk=1:n-1
    U(:,tk)=Tau(:,tk);
  x1(:,tk+1) =  x2(:,tk)*Ts + x1(:,tk);
      x2(:,tk+1) = (-inv(D(x1(:,tk)))*(transpose(G(x1(:,tk))))+ inv(D(x1(:,tk)))*Tau(:,tk))*Ts + x2(:,tk);

x3(:,tk)=   (-inv(D(x1(:,tk)))*(transpose(G(x1(:,tk))))+ inv(D(x1(:,tk)))*Tau(:,tk));


 Ac1 = [zeros(7,7), eye(7);
            -inv(D(x1(:,tk)))*L(x1(:,tk),x3(:,tk)) , zeros(7,7)];
    C= [zeros(7,7),eye(7)];

 Bc = [zeros(7,7);inv(D(x1(:,tk)))];
 sysc=ss(Ac1,Bc,C,0);
       sys=c2d(sysc,Ts);
       [A,B,C,D1]=ssdata(sys);
       
       
X=Ac1*[x1(:,tk);x2(:,tk)] + Bc*U(:,tk);
x_2(:,tk)=X(1:7);
x_3(:,tk)=X(8:14);


end 
figure
for j = 1:7
    subplot(4,2,j),plot(t_d1,x_2(j,:),t_d1,x2(j,1:n-1))
    title(['velocity: Joint',num2str(j)])
end
figure
for j = 1:7
    subplot(4,2,j),plot(t_d1,x_3(j,:),t_d1,x3(j,1:n-1))
    title(['acceleration: Joint',num2str(j)])
end
