function D =D(q);
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
            
             
             
for j=1:7,
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
    
    Q=zeros(4,4);
    
    Q=[0,-1,0,0;
     1,0,0,0;
     0,0,0,0;
     0,0,0,0];
   

   
U=zeros(4,4,7,7);
   
   U(:,:,1,1)= Q*T(:,:,1);
   U(:,:,2,1)= Q*T(:,:,1)*T(:,:,2);
   U(:,:,3,1)= Q*T(:,:,1)*T(:,:,2)*T(:,:,3);
   U(:,:,4,1)= Q*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4);
   U(:,:,5,1)= Q*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5);
   U(:,:,6,1)= Q*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6);
   U(:,:,7,1)= Q*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6)*T(:,:,7);
   
   
   U(:,:,2,2)= T(:,:,1)*Q*T(:,:,2);
   U(:,:,3,2)= T(:,:,1)*Q*T(:,:,2)*T(:,:,3);
   U(:,:,4,2)= T(:,:,1)*Q*T(:,:,2)*T(:,:,3)*T(:,:,4);
   U(:,:,5,2)= T(:,:,1)*Q*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5);
   U(:,:,6,2)= T(:,:,1)*Q*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6);
   U(:,:,7,2)= T(:,:,1)*Q*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6)*T(:,:,7);
   
  
   U(:,:,3,3)= T(:,:,1)*T(:,:,2)*Q*T(:,:,3);
   U(:,:,4,3)= T(:,:,1)*T(:,:,2)*Q*T(:,:,3)*T(:,:,4);
   U(:,:,5,3)= T(:,:,1)*T(:,:,2)*Q*T(:,:,3)*T(:,:,4)*T(:,:,5);
   U(:,:,6,3)= T(:,:,1)*T(:,:,2)*Q*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6);
   U(:,:,7,3)= T(:,:,1)*T(:,:,2)*Q*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6)*T(:,:,7);
   
   U(:,:,4,4)= T(:,:,1)*T(:,:,2)*T(:,:,3)*Q*T(:,:,4);
   U(:,:,5,4)= T(:,:,1)*T(:,:,2)*T(:,:,3)*Q*T(:,:,4)*T(:,:,5);
   U(:,:,6,4)= T(:,:,1)*T(:,:,2)*T(:,:,3)*Q*T(:,:,4)*T(:,:,5)*T(:,:,6);
   U(:,:,7,4)= T(:,:,1)*T(:,:,2)*T(:,:,3)*Q*T(:,:,4)*T(:,:,5)*T(:,:,6)*T(:,:,7);
   
   U(:,:,5,5)= T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*Q*T(:,:,5);
   U(:,:,6,5)= T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*Q*T(:,:,5)*T(:,:,6);
   U(:,:,7,5)= T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*Q*T(:,:,5)*T(:,:,6)*T(:,:,7);
   
   U(:,:,6,6)= T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*Q*T(:,:,6);
   U(:,:,7,6)= T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*Q*T(:,:,6)*T(:,:,7);
   
   U(:,:,7,7)= T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4)*T(:,:,5)*T(:,:,6)*Q*T(:,:,7);
   
   
   
  
   J=zeros(4,4,7);
 
   
   J(:,:,1)=[0.5*(-I(1,1,1)+I(2,2,1)+I(3,3,1)), I(1,2,1), I(1,3,1), m(1)*com(1,1);
             I(1,2,1) ,0.5*(I(1,1,1)-I(2,2,1)+I(3,3,1)), I(2,3,1), m(1)*com(2,1);
             I(1,3,1), I(2,3,1), 0.5*(I(1,1,1)+I(2,2,1)-I(3,3,1)), m(1)*com(3,1);
              m(1)*com(1,1), m(1)*com(2,1), m(1)*com(3,1), m(1)];
   
    J(:,:,2)=[0.5*(-I(1,1,2)+I(2,2,2)+I(3,3,2)), I(1,2,2), I(1,3,2), m(2)*com(1,2);
             I(1,2,2) ,0.5*(I(1,1,2)-I(2,2,2)+I(3,3,2)), I(2,3,2), m(2)*com(2,2);
             I(1,3,2), I(2,3,2), 0.5*(I(1,1,2)+I(2,2,2)-I(3,3,2)), m(2)*com(3,2);
              m(2)*com(1,2), m(2)*com(2,2), m(2)*com(3,2), m(2)];
   
          
     J(:,:,3)=[0.5*(-I(1,1,3)+I(2,2,3)+I(3,3,3)), I(1,2,3), I(1,3,3), m(3)*com(1,3);
             I(1,2,3) ,0.5*(I(1,1,3)-I(2,2,3)+I(3,3,3)), I(2,3,3), m(3)*com(2,3);
             I(1,3,3), I(2,3,3), 0.5*(I(1,1,3)+I(2,2,3)-I(3,3,3)), m(3)*com(3,3);
              m(3)*com(1,3), m(3)*com(2,3), m(3)*com(3,3), m(3)];      
   
   
    J(:,:,4)=[0.5*(-I(1,1,4)+I(2,2,4)+I(3,3,4)), I(1,2,4), I(1,3,4), m(4)*com(1,4);
             I(1,2,4) ,0.5*(I(1,1,4)-I(2,2,4)+I(3,3,4)), I(2,3,4), m(4)*com(2,4);
             I(1,3,4), I(2,3,4), 0.5*(I(1,1,4)+I(2,2,4)-I(3,3,4)), m(4)*com(3,4);
              m(4)*com(1,4), m(4)*com(2,4), m(4)*com(3,4), m(4)];
   
   
    J(:,:,5)=[0.5*(-I(1,1,5)+I(2,2,5)+I(3,3,5)), I(1,2,5), I(1,3,5), m(5)*com(1,5);
             I(1,2,5) ,0.5*(I(1,1,5)-I(2,2,5)+I(3,3,5)), I(2,3,5), m(5)*com(2,5);
             I(1,3,5), I(2,3,5), 0.5*(I(1,1,5)+I(2,2,5)-I(3,3,5)), m(5)*com(3,5);
              m(5)*com(1,5), m(5)*com(2,5), m(5)*com(3,5), m(5)];
   
   
    J(:,:,6)=[0.5*(-I(1,1,6)+I(2,2,6)+I(3,3,6)), I(1,2,6), I(1,3,6), m(6)*com(1,6);
             I(1,2,6) ,0.5*(I(1,1,6)-I(2,2,6)+I(3,3,6)), I(2,3,6), m(6)*com(2,6);
             I(1,3,6), I(2,3,6), 0.5*(I(1,1,6)+I(2,2,6)-I(3,3,6)), m(6)*com(3,6);
              m(6)*com(1,6), m(6)*com(2,6), m(6)*com(3,6), m(6)];
   
   
    J(:,:,7)=[0.5*(-I(1,1,7)+I(2,2,7)+I(3,3,7)), I(1,2,7), I(1,3,7), m(7)*com(1,7);
             I(1,2,7) ,0.5*(I(1,1,7)-I(2,2,7)+I(3,3,7)), I(2,3,7), m(7)*com(2,7);
             I(1,3,7), I(2,3,7), 0.5*(I(1,1,7)+I(2,2,7)-I(3,3,7)), m(7)*com(3,7);
              m(7)*com(1,7), m(7)*com(2,7), m(7)*com(3,7), m(7)];
          
          D=zeros(7,7);
   
   
    D(1,1)= trace(U(:,:,1,1)*J(:,:,1)*transpose(U(:,:,1,1))) + trace(U(:,:,2,1)*J(:,:,2)*transpose(U(:,:,2,1))) + trace(U(:,:,3,1)*J(:,:,3)*transpose(U(:,:,3,1))) + trace(U(:,:,4,1)*J(:,:,4)*transpose(U(:,:,4,1))) + trace(U(:,:,5,1)*J(:,:,5)*transpose(U(:,:,5,1))) + trace(U(:,:,6,1)*J(:,:,6)*transpose(U(:,:,6,1))) + trace(U(:,:,7,1)*J(:,:,7)*transpose(U(:,:,7,1))) ;
    D(1,2)= trace(U(:,:,2,2)*J(:,:,2)*transpose(U(:,:,2,1))) + trace(U(:,:,3,2)*J(:,:,3)*transpose(U(:,:,3,1))) + trace(U(:,:,4,2)*J(:,:,4)*transpose(U(:,:,4,1))) +trace(U(:,:,5,2)*J(:,:,5)*transpose(U(:,:,5,1))) + trace(U(:,:,6,2)*J(:,:,6)*transpose(U(:,:,6,1))) + trace(U(:,:,7,2)*J(:,:,7)*transpose(U(:,:,7,1))); 
    D(1,3)= trace(U(:,:,3,3)*J(:,:,3)*transpose(U(:,:,3,1)))+ trace(U(:,:,4,3)*J(:,:,4)*transpose(U(:,:,4,1)))+ trace(U(:,:,5,3)*J(:,:,5)*transpose(U(:,:,5,1)))+ trace(U(:,:,6,3)*J(:,:,6)*transpose(U(:,:,6,1)))+ trace(U(:,:,7,3)*J(:,:,7)*transpose(U(:,:,7,1))); 
    D(1,4)= trace(U(:,:,4,4)*J(:,:,4)*transpose(U(:,:,4,1)))+ trace(U(:,:,5,4)*J(:,:,5)*transpose(U(:,:,5,1)))+ trace(U(:,:,6,4)*J(:,:,6)*transpose(U(:,:,6,1)))+ trace(U(:,:,7,4)*J(:,:,7)*transpose(U(:,:,7,1)));
    D(1,5)= trace(U(:,:,5,5)*J(:,:,5)*transpose(U(:,:,5,1)))+ trace(U(:,:,6,5)*J(:,:,6)*transpose(U(:,:,6,1)))+ trace(U(:,:,7,5)*J(:,:,7)*transpose(U(:,:,7,1)));
    D(1,6)= trace(U(:,:,6,6)*J(:,:,6)*transpose(U(:,:,6,1)))+ trace(U(:,:,7,6)*J(:,:,7)*transpose(U(:,:,7,1)));
    D(1,7)= trace(U(:,:,7,7)*J(:,:,7)*transpose(U(:,:,7,1)));
    
    D(2,1)= trace(U(:,:,2,1)*J(:,:,2)*transpose(U(:,:,2,2)))+ trace(U(:,:,3,1)*J(:,:,3)*transpose(U(:,:,3,2)))+ trace(U(:,:,4,1)*J(:,:,4)*transpose(U(:,:,4,2)))+ trace(U(:,:,5,1)*J(:,:,5)*transpose(U(:,:,5,2)))+ trace(U(:,:,6,1)*J(:,:,6)*transpose(U(:,:,6,2)))+ trace(U(:,:,7,1)*J(:,:,7)*transpose(U(:,:,7,2)));
    D(2,2)= trace(U(:,:,2,2)*J(:,:,2)*transpose(U(:,:,2,2)))+ trace(U(:,:,3,2)*J(:,:,3)*transpose(U(:,:,3,2)))+ trace(U(:,:,4,2)*J(:,:,4)*transpose(U(:,:,4,2)))+ trace(U(:,:,5,2)*J(:,:,5)*transpose(U(:,:,5,2)))+ trace(U(:,:,6,2)*J(:,:,6)*transpose(U(:,:,6,2)))+ trace(U(:,:,7,2)*J(:,:,7)*transpose(U(:,:,7,2)));
    D(2,3)= trace(U(:,:,3,3)*J(:,:,3)*transpose(U(:,:,3,2)))+ trace(U(:,:,4,3)*J(:,:,4)*transpose(U(:,:,4,2)))+ trace(U(:,:,5,3)*J(:,:,5)*transpose(U(:,:,5,2)))+ trace(U(:,:,6,3)*J(:,:,6)*transpose(U(:,:,6,2)))+ trace(U(:,:,7,3)*J(:,:,7)*transpose(U(:,:,7,2))); 
    D(2,4)= trace(U(:,:,4,4)*J(:,:,4)*transpose(U(:,:,4,2)))+ trace(U(:,:,5,4)*J(:,:,5)*transpose(U(:,:,5,2)))+ trace(U(:,:,6,4)*J(:,:,6)*transpose(U(:,:,6,2)))+ trace(U(:,:,7,4)*J(:,:,7)*transpose(U(:,:,7,2))); 
    D(2,5)= trace(U(:,:,5,5)*J(:,:,5)*transpose(U(:,:,5,2)))+ trace(U(:,:,6,5)*J(:,:,6)*transpose(U(:,:,6,2)))+ trace(U(:,:,7,5)*J(:,:,7)*transpose(U(:,:,7,2)));
    D(2,6)= trace(U(:,:,6,6)*J(:,:,6)*transpose(U(:,:,6,2)))+ trace(U(:,:,7,6)*J(:,:,7)*transpose(U(:,:,7,2))); 
    D(2,7)= trace(U(:,:,7,7)*J(:,:,7)*transpose(U(:,:,7,2)));
  
    D(3,1)= trace(U(:,:,3,1)*J(:,:,3)*transpose(U(:,:,3,3)))+ trace(U(:,:,4,1)*J(:,:,4)*transpose(U(:,:,4,3)))+ trace(U(:,:,5,1)*J(:,:,5)*transpose(U(:,:,5,3)))+ trace(U(:,:,6,1)*J(:,:,6)*transpose(U(:,:,6,3)))+ trace(U(:,:,7,1)*J(:,:,7)*transpose(U(:,:,7,3)));
    D(3,2)= trace(U(:,:,3,2)*J(:,:,3)*transpose(U(:,:,3,3)))+ trace(U(:,:,4,2)*J(:,:,4)*transpose(U(:,:,4,3)))+ trace(U(:,:,5,2)*J(:,:,5)*transpose(U(:,:,5,3)))+ trace(U(:,:,6,2)*J(:,:,6)*transpose(U(:,:,6,3)))+ trace(U(:,:,7,2)*J(:,:,7)*transpose(U(:,:,7,3)));
    D(3,3)= trace(U(:,:,3,3)*J(:,:,3)*transpose(U(:,:,3,3)))+ trace(U(:,:,4,3)*J(:,:,4)*transpose(U(:,:,4,3)))+ trace(U(:,:,5,3)*J(:,:,5)*transpose(U(:,:,5,3)))+ trace(U(:,:,6,3)*J(:,:,6)*transpose(U(:,:,6,3)))+ trace(U(:,:,7,3)*J(:,:,7)*transpose(U(:,:,7,3)));
    D(3,4)= trace(U(:,:,4,4)*J(:,:,4)*transpose(U(:,:,4,3)))+ trace(U(:,:,5,4)*J(:,:,5)*transpose(U(:,:,5,3)))+ trace(U(:,:,6,4)*J(:,:,6)*transpose(U(:,:,6,3)))+ trace(U(:,:,7,4)*J(:,:,7)*transpose(U(:,:,7,3)));
    D(3,5)= trace(U(:,:,5,5)*J(:,:,5)*transpose(U(:,:,5,3)))+ trace(U(:,:,6,5)*J(:,:,6)*transpose(U(:,:,6,3)))+ trace(U(:,:,7,5)*J(:,:,7)*transpose(U(:,:,7,3)));
    D(3,6)= trace(U(:,:,6,6)*J(:,:,6)*transpose(U(:,:,6,3)))+ trace(U(:,:,7,6)*J(:,:,7)*transpose(U(:,:,7,3)));
    D(3,7)= trace(U(:,:,7,7)*J(:,:,7)*transpose(U(:,:,7,3)));
     
     
    D(4,1)= trace(U(:,:,4,1)*J(:,:,4)*transpose(U(:,:,4,4)))+ trace(U(:,:,5,1)*J(:,:,5)*transpose(U(:,:,5,4)))+ trace(U(:,:,6,1)*J(:,:,6)*transpose(U(:,:,6,4)))+ trace(U(:,:,7,1)*J(:,:,7)*transpose(U(:,:,7,4))); 
    D(4,2)= trace(U(:,:,4,2)*J(:,:,4)*transpose(U(:,:,4,4)))+ trace(U(:,:,5,2)*J(:,:,5)*transpose(U(:,:,5,4)))+ trace(U(:,:,6,2)*J(:,:,6)*transpose(U(:,:,6,4)))+ trace(U(:,:,7,2)*J(:,:,7)*transpose(U(:,:,7,4))); 
    D(4,3)= trace(U(:,:,4,3)*J(:,:,4)*transpose(U(:,:,4,4)))+ trace(U(:,:,5,3)*J(:,:,5)*transpose(U(:,:,5,4)))+ trace(U(:,:,6,3)*J(:,:,6)*transpose(U(:,:,6,4)))+ trace(U(:,:,7,3)*J(:,:,7)*transpose(U(:,:,7,4)));
    D(4,4)= trace(U(:,:,4,4)*J(:,:,4)*transpose(U(:,:,4,4)))+ trace(U(:,:,5,4)*J(:,:,5)*transpose(U(:,:,5,4)))+ trace(U(:,:,6,4)*J(:,:,6)*transpose(U(:,:,6,4)))+ trace(U(:,:,7,4)*J(:,:,7)*transpose(U(:,:,7,4)));
    D(4,5)= trace(U(:,:,5,5)*J(:,:,5)*transpose(U(:,:,5,4)))+ trace(U(:,:,6,5)*J(:,:,6)*transpose(U(:,:,6,4)))+ trace(U(:,:,7,5)*J(:,:,7)*transpose(U(:,:,7,4)));
    D(4,6)= trace(U(:,:,6,6)*J(:,:,6)*transpose(U(:,:,6,4)))+ trace(U(:,:,7,6)*J(:,:,7)*transpose(U(:,:,7,4))); 
    D(4,7)= trace(U(:,:,7,7)*J(:,:,7)*transpose(U(:,:,7,4)));
    
    D(5,1)= trace(U(:,:,5,1)*J(:,:,5)*transpose(U(:,:,5,5)))+ trace(U(:,:,6,1)*J(:,:,6)*transpose(U(:,:,6,5)))+ trace(U(:,:,7,1)*J(:,:,7)*transpose(U(:,:,7,5)));
    D(5,2)= trace(U(:,:,5,2)*J(:,:,5)*transpose(U(:,:,5,5)))+ trace(U(:,:,6,2)*J(:,:,6)*transpose(U(:,:,6,5)))+ trace(U(:,:,7,2)*J(:,:,7)*transpose(U(:,:,7,5)));
    D(5,3)= trace(U(:,:,5,3)*J(:,:,5)*transpose(U(:,:,5,5)))+ trace(U(:,:,6,3)*J(:,:,6)*transpose(U(:,:,6,5)))+ trace(U(:,:,7,3)*J(:,:,7)*transpose(U(:,:,7,5))); 
    D(5,4)= trace(U(:,:,5,4)*J(:,:,5)*transpose(U(:,:,5,5)))+ trace(U(:,:,6,4)*J(:,:,6)*transpose(U(:,:,6,5)))+ trace(U(:,:,7,4)*J(:,:,7)*transpose(U(:,:,7,5))); 
    D(5,5)= trace(U(:,:,5,5)*J(:,:,5)*transpose(U(:,:,5,5)))+ trace(U(:,:,6,5)*J(:,:,6)*transpose(U(:,:,6,5)))+ trace(U(:,:,7,5)*J(:,:,7)*transpose(U(:,:,7,5)));
    D(5,6)= trace(U(:,:,6,6)*J(:,:,6)*transpose(U(:,:,6,5)))+ trace(U(:,:,7,6)*J(:,:,7)*transpose(U(:,:,7,5))); 
    D(5,7)= trace(U(:,:,7,7)*J(:,:,7)*transpose(U(:,:,7,5)));
    
    D(6,1)= trace(U(:,:,6,1)*J(:,:,6)*transpose(U(:,:,6,6)))+ trace(U(:,:,7,1)*J(:,:,7)*transpose(U(:,:,7,6)));
    D(6,2)= trace(U(:,:,6,2)*J(:,:,6)*transpose(U(:,:,6,6)))+ trace(U(:,:,7,2)*J(:,:,7)*transpose(U(:,:,7,6)));
    D(6,3)= trace(U(:,:,6,3)*J(:,:,6)*transpose(U(:,:,6,6)))+ trace(U(:,:,7,3)*J(:,:,7)*transpose(U(:,:,7,6))); 
    D(6,4)= trace(U(:,:,6,4)*J(:,:,6)*transpose(U(:,:,6,6)))+ trace(U(:,:,7,4)*J(:,:,7)*transpose(U(:,:,7,6))); 
    D(6,5)= trace(U(:,:,6,5)*J(:,:,6)*transpose(U(:,:,6,6)))+ trace(U(:,:,7,5)*J(:,:,7)*transpose(U(:,:,7,6))); 
    D(6,6)= trace(U(:,:,6,6)*J(:,:,6)*transpose(U(:,:,6,6)))+ trace(U(:,:,7,6)*J(:,:,7)*transpose(U(:,:,7,6))); 
    D(6,7)= trace(U(:,:,7,7)*J(:,:,7)*transpose(U(:,:,7,6)));
    
    D(7,1)= trace(U(:,:,7,1)*J(:,:,7)*transpose(U(:,:,7,7)));
    D(7,2)= trace(U(:,:,7,2)*J(:,:,7)*transpose(U(:,:,7,7)));
    D(7,3)= trace(U(:,:,7,3)*J(:,:,7)*transpose(U(:,:,7,7)));
    D(7,4)= trace(U(:,:,7,4)*J(:,:,7)*transpose(U(:,:,7,7)));
    D(7,5)= trace(U(:,:,7,5)*J(:,:,7)*transpose(U(:,:,7,7)));
    D(7,6)= trace(U(:,:,7,6)*J(:,:,7)*transpose(U(:,:,7,7)));
    D(7,7)= trace(U(:,:,7,7)*J(:,:,7)*transpose(U(:,:,7,7)));
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