function x = statespace(q,qd,tau)
DH_a = [0 0 0.045 -0.045 0 0 0];
    DH_d = [0 0 .55 0 0.3 0 0.061];
    DH_alpha = [-pi/2 pi/2 -pi/2 pi/2 -pi/2 pi/2 0];

    m = [8.3 4.8484 1.7251 2.17266212 0.35655692 0.40915886 0.07548270];
    
     
    
    com = zeros(3,7);
    com(:,1) = [0.3506e-3; 132.6795e-3; 0.6286e-3];
    com(:,2) = [-0.2230e-3; -21.3924e-3; 13.3754e-3];
    com(:,3) = [-38.7565e-3; 217.9078e-3; 0.0252e-3];
    com(:,4) = [0.00553408;0.00006822;0.11927695];
    com(:,5) = [0.00005483;0.02886286;0.00148493];
    com(:,6) = [-0.00005923;-0.07007252;-0.00335185];
    com(:,7) = [ 0.00014836;0.00007252;-0.00335185];
     
        
    
    I = zeros(3,3,7);
 I(:,:,1) = [ 242919.8516e-6,  -636.5542e-6,  93.16877e-6 ;
                 -636.5542e-6  , 92036.7003e-6,  -262.6542e-6 ;
                  93.16877e-6 , -262.6542e-6 , 207050.7372e-6 ];
    I(:,:,2) = [ 32413.1852e-6,  20.2663e-6,  143.7579e-3;
                  20.2663e-6 , 21649.2574e-6, -38.6737e-6 ;
                  143.7579e-3,  -38.6737e-6, 25026.5101e-6 ];
    I(:,:,3) = [ 138575.0448e-6, 16890.4832e-6 ,    -6.5293e-6 ;
                16890.4832e-6 ,  5749.2209e-6,  -7.1669e-6 ;
                  -6.5293e-6  ,  -7.1669e-6 , 141310.5180e-6 ];
    I(:,:,4) = [  35752.4193e-6, -14.5876e-6 , -62.1491e-6 ;
                  -14.5876e-6 ,  35769.9341-6,  -29.2508e-6 ;
                -62.1491e-6 , 29.2508e-6 ,  2.9213314e-6 ];
    I(:,:,5) = [ 567.9724e-6,-0.4416e-6  , 0.0045e-6 ;
                -0.4416e-6 ,  172.3759e-6, -0.8172e-6 ;
                0.0045e-6, -0.8172e-6,  597.6265e-6 ];
    I(:,:,6) =  [ 979.8079e-6 ,  -0.2804e-6 , 0.5308e-6 ;
                   -0.2804e-6,  550.3998e-6, -225.3322e-6 ;
                  0.5308e-6, -225.3322e-6,  602.4224e-6 ];
  
        I(:,:,7) = [4536.8120e-6 , 0.00000019, -2.8944e-6  ;      % WAM without hand
                0, 4537.0156e-6, 0.00000000 ;
                -2.8944e-6  , 0.00000000, 42.1994e-6 ];
             T = zeros(4,4,7);
             
for j=1:7,
        a = DH_a(j);
        d = DH_d(j);
        alpha = DH_alpha(j);
        T(:,:,j) = getTransformDH(a, d, alpha, q(j));
end 
A = zeros(4,4,7);
A(:,:,1) = T(:,:,1);
A(:,:,2) = A(:,:,1)*T(:,:,2);
A(:,:,3) = A(:,:,2)*T(:,:,3);
A(:,:,4) = A(:,:,3)*T(:,:,4);
A(:,:,5) = A(:,:,4)*T(:,:,5);
A(:,:,6) = A(:,:,5)*T(:,:,6);
A(:,:,7) = A(:,:,6)*T(:,:,7);

R = zeros(4,4,7,7);
for i=1:7,
    for p=1:7,
        if i <= p
            if i == 1,  
                R(:,:,p,i)= A(:,:,p);
            else 
 
            b=i-1;
            
            R(:,:,p,i)= inv(A(:,:,b))*A(:,:,p);
            end 
        else 
         end 
    end 
end 
Dx = zeros(3,7,7);
Dy = zeros(3,7,7);
Dz = zeros(3,7,7);
Dstar = zeros(3,7,7);
sigma = zeros(3,7,7);
   for i=1:7
       for j=1:7,
           p=max(i,j);
           zeta=[0;0;1];
           detta=[0;0;1];
Dx(i,p) = dot(R((1:3),1,p,i),cross(zeta,R((1:3),4,p,i))); 
Dy(i,p) = dot(R((1:3),2,p,i),cross(zeta,R((1:3),4,p,i)));
Dz(i,p) = dot(R((1:3),3,p,i),cross(zeta,R((1:3),4,p,i)));
Dstar(:,i,p) = [Dx(i,p); Dy(i,p) ; Dz(i,p)];
sigma(:,i,p) = [R(3,1,p,i);R(3,2,p,i);R(3,3,p,i)];


 
       end 
   end
   E = zeros(7,7,7);
   D = zeros(7,7);
   for i=1:7,
       for j=1:7,
           p = max(i,j);
           for x=p:7,
               
   E(i,j,x) = m(x)*(transpose([sigma(:,i,x)])*transpose(T(1:3,1:3,x))*I(:,:,x)*T(1:3,1:3,x)*([sigma(:,j,x)]) + sum(Dstar(:,i,x).*Dstar(:,j,x),1) +sum(com(:,x).*(cross(Dstar(:,i,x),sigma(:,j,x)) + cross(Dstar(:,j,x),sigma(:,i,x))),1)  );
           end 
       D(i,j)=  E(i,j,1)+  E(i,j,2)+  E(i,j,3)+  E(i,j,4)+ E(i,j,5)+  E(i,j,6)+  E(i,j,7);
       end
   end
   comstar = zeros(4,7);
    comstar(:,1) = [0.3506e-3; 132.6795e-3; 0.6286e-3;1];
    comstar(:,2) = [-0.2230e-3; -21.3924e-3; 13.3754e-3;1];
    comstar(:,3) = [-38.7565e-3; 217.9078e-3; 0.0252e-3;1];
    comstar(:,4) = [0.00553408;0.00006822;0.11927695;1];
    comstar(:,5) = [0.00005483;0.02886286;0.00148493;1];
    comstar(:,6) = [-0.00005923;-0.01686123;0.02419052;1];
    comstar(:,7) = [ com(:,7);1];
  Q=[0,-1,0,0;
     1,0,0,0;
     0,0,0,0;
     0,0,0,0];
   gravity=[0,0,-9.81,0];
   G(1)= -m(1)*gravity*Q*T(:,:,1)*comstar(:,1)-m(2)*gravity*Q*T(:,:,1)*T(:,:,2)*comstar(:,2)-m(3)*gravity*Q*T(:,:,1)*T(:,:,2)*T(:,:,3)*comstar(:,3)-m(4)*gravity*Q*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*comstar(:,4)-m(5)*gravity*Q*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*comstar(:,5)-m(6)*gravity*Q*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6)*comstar(:,6)-m(7)*gravity*Q*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6)*T(:,:,7)*comstar(:,7);   
   G(2)= -m(2)*gravity*T(:,:,1)*Q*T(:,:,2)*comstar(:,2)-m(3)*gravity*T(:,:,1)*Q*T(:,:,2)*T(:,:,3)*comstar(:,3)-m(4)*gravity*T(:,:,1)*Q*T(:,:,2)*T(:,:,3)*T(:,:,4)*comstar(:,4)-m(5)*gravity*T(:,:,1)*Q*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*comstar(:,5)-m(6)*gravity*T(:,:,1)*Q*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6)*comstar(:,6)-m(7)*gravity*T(:,:,1)*Q*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6)*T(:,:,7)*comstar(:,7);
   G(3)= -m(3)*gravity*T(:,:,1)*T(:,:,2)*Q*T(:,:,3)*comstar(:,3)-m(4)*gravity*T(:,:,1)*T(:,:,2)*Q*T(:,:,3)*T(:,:,4)*comstar(:,4)-m(5)*gravity*T(:,:,1)*T(:,:,2)*Q*T(:,:,3)*T(:,:,4)*T(:,:,5)*comstar(:,5)-m(6)*gravity*T(:,:,1)*T(:,:,2)*Q*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6)*comstar(:,6)-m(7)*gravity*T(:,:,1)*T(:,:,2)*Q*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6)*T(:,:,7)*comstar(:,7);
   G(4)= -m(4)*gravity*T(:,:,1)*T(:,:,2)*T(:,:,3)*Q*T(:,:,4)*comstar(:,4)-m(5)*gravity*T(:,:,1)*T(:,:,2)*T(:,:,3)*Q*T(:,:,4)*T(:,:,5)*comstar(:,5)-m(6)*gravity*T(:,:,1)*T(:,:,2)*T(:,:,3)*Q*T(:,:,4)*T(:,:,5)*T(:,:,6)*comstar(:,6)-m(7)*gravity*T(:,:,1)*T(:,:,2)*T(:,:,3)*Q*T(:,:,4)*T(:,:,5)*T(:,:,6)*T(:,:,7)*comstar(:,7);
   G(5)= -m(5)*gravity*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*Q*T(:,:,5)*comstar(:,5)-m(6)*gravity*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*Q*T(:,:,5)*T(:,:,6)*comstar(:,6)-m(7)*gravity*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*Q*T(:,:,5)*T(:,:,6)*T(:,:,7)*comstar(:,7);
   G(6)= -m(6)*gravity*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*Q*T(:,:,6)*comstar(:,6)-m(7)*gravity*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*Q*T(:,:,6)*T(:,:,7)*comstar(:,7);
   G(7)= -m(7)*gravity*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6)*Q*T(:,:,7)*comstar(:,7);
   
   
   
   
   
xd = [qd;-inv(D)*transpose(G)] + [zeros(7,7);inv(D)]*tau;
x= xd*0.002 + [q;qd];

end        
             
 function T = getTransformDH(a,d,alpha,q)
    ct = cos(q);
    st = sin(q);
    cb = cos(alpha);
    sb = sin(alpha);
    
    T = [ ct, -st*cb,  st*sb, a*ct;
          st,  ct*cb, -ct*sb, a*st;
          0, sb, cb, d;
          0, 0, 0, 1 ];            
 end       