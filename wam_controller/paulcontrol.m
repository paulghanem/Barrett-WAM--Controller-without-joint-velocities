function tau = paulcontrol(q,qdd)

DH_a = [0 0 0.045 -0.045 0 0 0];
    DH_d = [0 0 .55 0 0.3 0 0.061];
    DH_alpha = [-pi/2 pi/2 -pi/2 pi/2 -pi/2 pi/2 0];

    m = [8.3 4.8487 1.7251 2.17266212 0.35655692 0.40915886 0.07548270];
    
     
    
    com = zeros(3,7);
    com(:,1) = [0.3506e-3; 132.6795e-3; 0.6286e-3];
    com(:,2) = [-0.2230e-3; -21.3924e-3; 13.3754e-3];
    com(:,3) = [-38.7565e-3; 217.9078e-3; 0.0252e-3];
    com(:,4) = [0.00553408;0.00006822;0.11927695];
    com(:,5) = [0.00005483;0.02886286;0.00148493];
    com(:,6) = [-0.00005923;-0.07007252;-0.00335185];
    com(:,7) = [ 0.00014836;0.00007252;-0.00335185];
     hand_mass = 1.3731;  %for Schunk Gripper
      DH_d(7) = DH_d(7) + .0953; % Adding the distance to the hand frame for Schunk Gripper
     com(:,7) = ([0.001127;	-0.000211;	0.054851].*hand_mass + com(:,7).*m(7))/(hand_mass + m(7)); % Finding the combined center of mass
        
    
    I = zeros(3,3,7);
    I(:,:,1) = [ 95157.4294e-6,   246.1404e-6,   -95.0183e-6 ;
                   246.1404e-6, 92032.3524e-6,  -962.6725e-6 ;
                   -95.0183e-6,  -962.6725e-6, 59290.5997e-6 ];
    I(:,:,2) = [ 29326.8098e-6,   -43.3994e-6,  -129.2942e-6 ;
                   -43.3994e-6, 20781.5826e-6,  1348.6924e-6 ;
                  -129.2942e-6,  1348.6924e-6, 22807.3271e-6 ];
    I(:,:,3) = [ 56662.2970e-6, -2321.6892e-6,     8.2125e-6 ;
                 -2321.6892e-6,  3158.0509e-6,   -16.6307e-6 ;
                     8.2125e-6,   -16.6307e-6, 56806.6024e-6 ];
    I(:,:,4) = [  0.01067491,  0.00004503, -0.00135557 ;
                  0.00004503,  0.01058659, -0.00011002 ;
                 -0.00135557, -0.00011002,  0.00282036 ];
    I(:,:,5) = [  0.00037112, -0.00000008, -0.00000003 ;
                 -0.00000008,  0.00019434, -0.00001613 ;
                 -0.00000003, -0.00001613,  0.00038209 ];
    I(:,:,6) =  [  0.00054889,  0.00000019, -0.00000010 ;
                   0.00000019,  0.00023846, -0.00004430 ;
                  -0.00000010, -0.00004430,  0.00045133 ];
    I(:,:,7) = [ 0.003167342,          0,         0;       % Approximate Inertia Tensor for Schunk Gripper
                              0, 0.001128942,         0;
                              0,          0, 0.00258007;];
            
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
   for i=1:7,
       for j=1:7,
           p=max(i,j);
           
Dx(i,p) = -R(1,1,p,i)*R(2,4,p,i) + R(2,1,p,i)*R(1,4,p,i); 
Dy(i,p) = -R(1,2,p,i)*R(2,4,p,i) + R(2,2,p,i)*R(1,4,p,i);
Dz(i,p) = -R(1,3,p,i)*R(2,4,p,i) + R(2,3,p,i)*R(1,4,p,i);
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
               
   E(i,j,x) = m(x)*(transpose(sigma(:,i,x))*I(:,:,x)*sigma(:,j,x) +sum(Dstar(:,i,x).*Dstar(:,j,x,1),1) + sum(com(:,x).*(cross(Dstar(:,i,x),sigma(:,j,x)) + cross(Dstar(:,j,x),sigma(:,i,x))),1));
           end 
       D(i,j)=  E(i,j,1)+  E(i,j,2)+  E(i,j,3)+  E(i,j,4)+ E(i,j,5)+  E(i,j,6)+  E(i,j,7);
       end
   end
   grav = zeros(7,4);
   for i=1:7,
       if i==1 
           grav(i,:)= [0,0,-9.81,0];
       else 
   grav(i,:) =  [0,0,-9.81,0]*A(i-1)*[0,-1,0,0;1,0,0,0;0,0,0,0;0,0,0,0];
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
   h = zeros(4,7);
   hstar = zeros(4,7);
   G = zeros(1,7);
   for i=1:7, 
      for p=i:7,
          h(:,p)=m(p)*inv(A(:,:,i))*A(:,:,p)*comstar(:,p);
      end 
      hstar(:,i) = sum(h,2);
      G(i) = grav(i,:)*hstar(:,i);
   end 
   tau = zeros(7,1);

tau = D*qdd + transpose(G)  ;

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