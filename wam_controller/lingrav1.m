function C2=lingrav1(q)


DH_a = [0 0 0.045 -0.045 0 0 0];
    DH_d = [0 0 .55 0 0.45 0 0.061];
    DH_alpha = [-pi/2 pi/2 -pi/2 pi/2 -pi/2 pi/2 0];

    m = [10.7676 3.87 1.8022 2.4 0.1237 0.4979886 0.07548270];
    
     
    com= zeros(3,7);
    
   com(:,1) = [-0.00443422;0.12189039;-0.00066489];
    com(:,2) = [-0.00236983;0.03105614;0.01542114 ];
    com(:,3) = [-0.03825858;0.20750770;0.00003309];
    com(:,4) = [0.00498512;-0.00022942;0.13271662 ];
    com(:,5) = [0.00008921;0.00511217;0.00435824];
    com(:,6) = [-0.00012262;-0.01703194;0.02468336];
    com(:,7) = [-0.00007974;0.00016313;-0.00323552 ];
     
     % hand_mass =1.385;
      %  DH_d(7) =DH_d(7)+ .0953; % Adding the distance to the hand frame for the Schunk Gripper
       % com(:,7) =(com(:,7).*hand_mass+com(:,7).*m(7))/(hand_mass+m(7)); % Finding the combined center of mass
        %m(7) =m(7)+hand_mass;  
        
    I=zeros(3,3,7);
    T=zeros(4,4);
     I(:,:,1) = [0.29486350,-0.00795023,-0.00009311;
                 -0.00795023,0.11350017,-0.00018711;
                -0.00009311,-0.00018711,0.25065343];
    I(:,:,2) =[0.02606840,-0.00001346,-0.00011701;
               -0.00001346,0.01472202,0.00003659;
              -0.00011701,0.00003659,0.01934814];
    I(:,:,3) = [0.13671601,-0.01680434, 0.00000510;
               -0.01680434,0.00588354,-0.00000530;
                0.00000510,-0.00000530,0.13951371];
    I(:,:,4) = [ 0.05719268,0.00001467,0.00008193;
                 0.00001467,0.05716470,-0.00009417;
                 0.00008193,-0.00009417,0.00300441] ;
    I(:,:,5) =[0.00005587,0.00000026,.00000000;
               0.00000026,0.00007817,-0.00000083;
               0.00000000,-0.00000083,0.00006594];
    I(:,:,6) = [0.00093106,0.00000148,-0.00000201;
                0.00000148, 0.00049833,-0.00022162;
               -0.00000201,-0.00022162,0.00057483];
    I(:,:,7) = [0.003167342, 0,0;       % Approximate Inertia Tensor for Schunk Gripper
                0,0.001128942,0;
                 0,0,0.00258007;];  
            
             
for j=1:4,
        a = DH_a(j);
        d = DH_d(j);
        alpha = DH_alpha(j);
        T(:,:,j) = getTransformDH(a, d, alpha, q(j));
end 


comstar=zeros(4,7);

   
    comstar(:,1) = [com(:,1);1];
    comstar(:,2) = [com(:,2);1];
    comstar(:,3) = [com(:,3);1];
    comstar(:,4) = [com(:,4);1];
    comstar(:,5) = [com(:,5);1];
    comstar(:,6) = [com(:,6);1];
    comstar(:,7) = [com(:,7);1];
    
   
    
    Q=[0,-1,0,0;
     1,0,0,0;
     0,0,0,0;
     0,0,0,0];
    
   
    gravity=[0,0,-9.81,0];
    
    
   C2(1,1)=-m(1)*gravity*Q*Q*T(:,:,1)*comstar(:,1)-m(2)*gravity*Q*Q*T(:,:,1)*T(:,:,2)*comstar(:,2)-m(3)*gravity*Q*Q*T(:,:,1)*T(:,:,2)*T(:,:,3)*comstar(:,3)-m(4)*gravity*Q*Q*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*comstar(:,4);   
   
   C2(1,2)=-m(2)*gravity*Q*T(:,:,1)*Q*T(:,:,2)*comstar(:,2)-m(3)*gravity*Q*T(:,:,1)*Q*T(:,:,2)*T(:,:,3)*comstar(:,3)- m(4)*gravity*Q*T(:,:,1)*Q*T(:,:,2)*T(:,:,3)*T(:,:,4)*comstar(:,4);
   
   C2(1,3)=-m(3)*gravity*Q*T(:,:,1)*T(:,:,2)*Q*T(:,:,3)*comstar(:,3)-   m(4)*gravity*Q*T(:,:,1)*T(:,:,2)*Q*T(:,:,3)*T(:,:,4)*comstar(:,4);
 
   C2(1,4)=-m(4)*gravity*Q*T(:,:,1)*T(:,:,2)*T(:,:,3)*Q*T(:,:,4)*comstar(:,4);
   
   
   C2(2,1)=-m(2)*gravity*Q*T(:,:,1)*Q*T(:,:,2)*comstar(:,2)-   m(3)*gravity*Q*T(:,:,1)*Q*T(:,:,2)*T(:,:,3)*comstar(:,3)-   m(4)*gravity*Q*T(:,:,1)*Q*T(:,:,2)*T(:,:,3)*T(:,:,4)*comstar(:,4);
   
   C2(2,2)=-m(2)*gravity*T(:,:,1)*Q*Q*T(:,:,2)*comstar(:,2)-   m(3)*gravity*T(:,:,1)*Q*Q*T(:,:,2)*T(:,:,3)*comstar(:,3)-   m(4)*gravity*T(:,:,1)*Q*Q*T(:,:,2)*T(:,:,3)*T(:,:,4)*comstar(:,4);
   
   C2(2,3)=-m(3)*gravity*T(:,:,1)*Q*T(:,:,2)*Q*T(:,:,3)*comstar(:,3)-   m(4)*gravity*T(:,:,1)*Q*T(:,:,2)*Q*T(:,:,3)*T(:,:,4)*comstar(:,4);
   
   C2(2,4)=-m(4)*gravity*T(:,:,1)*Q*T(:,:,2)*T(:,:,3)*Q*T(:,:,4)*comstar(:,4);
  
   
   
   C2(3,1)=-m(3)*gravity*Q*T(:,:,1)*T(:,:,2)*Q*T(:,:,3)*comstar(:,3)-   m(4)*gravity*Q*T(:,:,1)*T(:,:,2)*Q*T(:,:,3)*T(:,:,4)*comstar(:,4);
   
   C2(3,2)=-m(3)*gravity*T(:,:,1)*Q*T(:,:,2)*Q*T(:,:,3)*comstar(:,3)-   m(4)*gravity*T(:,:,1)*Q*T(:,:,2)*Q*T(:,:,3)*T(:,:,4)*comstar(:,4);
   
   C2(3,3)=-m(3)*gravity*T(:,:,1)*T(:,:,2)*Q*Q*T(:,:,3)*comstar(:,3)-   m(4)*gravity*T(:,:,1)*T(:,:,2)*Q*Q*T(:,:,3)*T(:,:,4)*comstar(:,4);
   
   C2(3,4)= -m(4)*gravity*T(:,:,1)*T(:,:,2)*Q*T(:,:,3)*Q*T(:,:,4)*comstar(:,4);
   
   
   C2(4,1)=C2(1,4);
   C2(4,2)=C2(2,4);
   C2(4,3)=C2(3,4);
   C2(4,4)= -m(4)*gravity*T(:,:,1)*T(:,:,2)*T(:,:,3)*Q*Q*T(:,:,4)*comstar(:,4);
   
   
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

   