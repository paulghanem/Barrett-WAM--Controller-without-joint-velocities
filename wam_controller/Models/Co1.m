function Co=Co1(q,qd)
DH_a = [0 0 0.045 -0.045 ];
    DH_d = [0 0 .55 0 ];
    DH_alpha = [-pi/2 pi/2 -pi/2 pi/2];

    m = [10.7676 3.87 1.8022 2.4];
    Q=[0,-1,0,0;
     1,0,0,0;
     0,0,0,0;
     0,0,0,0];
     
    
    com = zeros(3,4);
   com(:,1) = [-0.00443422; 0.12189039; -0.00066489];
    com(:,2) = [-0.00236983; 0.03105614; 0.01542114 ];
    com(:,3) = [-0.03825858; 0.20750770; 0.00003309];
    com(:,4) = [ 0.00498512;-0.00022942;0.13271662 ];
    
     
     
    
     I(:,:,1) = [ 0.29486350 , -0.00795023 ,-0.00009311;
                 -0.00795023 , 0.11350017 , -0.00018711;
                -0.00009311 , -0.00018711 , 0.25065343];
    I(:,:,2) =[0.02606840 ,-0.00001346 , -0.00011701;
               -0.00001346,  0.01472202 ,0.00003659;
               -0.00011701 , 0.00003659,  0.01934814];
    I(:,:,3) = [0.13671601,  -0.01680434,  0.00000510;
               -0.01680434 , 0.00588354 , -0.00000530;
                0.00000510 , -0.00000530 , 0.13951371];
    I(:,:,4) = [ 0.05719268 , 0.00001467 , 0.00008193;
                 0.00001467 , 0.05716470 , -0.00009417;
                 0.00008193 , -0.00009417 , 0.00300441] ;
   
             T = zeros(4,4,4);
             
for j=1:4,
        a = DH_a(j);
        d = DH_d(j);
        alpha = DH_alpha(j);
        T(:,:,j) = getTransformDH(a, d, alpha, q(j));
end 
    
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
   
   
  
          
    
   F(:,:,1,1)= Q*T(:,:,1);
   F(:,:,2,1)= Q*T(:,:,1)*T(:,:,2);
   F(:,:,3,1)= Q*T(:,:,1)*T(:,:,2)*T(:,:,3);
   F(:,:,4,1)= Q*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4);
   
   
   F(:,:,2,2)= T(:,:,1)*Q*T(:,:,2);
   F(:,:,3,2)= T(:,:,1)*Q*T(:,:,2)*T(:,:,3);
   F(:,:,4,2)= T(:,:,1)*Q*T(:,:,2)*T(:,:,3)*T(:,:,4);
   
  
   F(:,:,3,3)= T(:,:,1)*T(:,:,2)*Q*T(:,:,3);
   F(:,:,4,3)= T(:,:,1)*T(:,:,2)*Q*T(:,:,3)*T(:,:,4);
   
   F(:,:,4,4)= T(:,:,1)*T(:,:,2)*T(:,:,3)*Q*T(:,:,4);
   
    
    
    
    
    
    
   U(:,:,1,1,1)= Q*Q*T(:,:,1);
   U(:,:,1,1,2)= 0;
   U(:,:,1,1,3)= 0;
   U(:,:,1,1,4)= 0;
   
   
   U(:,:,1,2,1)= 0;
   U(:,:,1,2,2)= 0;
   U(:,:,1,2,3)= 0;
   U(:,:,1,2,4)= 0;
   
   U(:,:,1,3,1)= 0;
   U(:,:,1,3,2)= 0;
   U(:,:,1,3,3)= 0;
   U(:,:,1,3,4)= 0;
   
   
   
   U(:,:,1,4,1)= 0;
   U(:,:,1,4,2)= 0;
   U(:,:,1,4,3)= 0;
   U(:,:,1,4,4)= 0;
   
   
   
   
   
   U(:,:,2,1,1)= Q*Q*T(:,:,1)*T(:,:,2);
   U(:,:,2,1,2)= Q*T(:,:,1)*Q*T(:,:,2);
   U(:,:,2,1,3)=0 ;
   U(:,:,2,1,4)=0 ;
   
   
   U(:,:,2,2,1)= Q*T(:,:,1)*Q*T(:,:,2);
   U(:,:,2,2,2)= T(:,:,1)*Q*Q*T(:,:,2);
   U(:,:,2,2,3)= 0;
   U(:,:,2,2,4)= 0;
   
   
   U(:,:,2,3,1)= 0;
   U(:,:,2,3,2)= 0;
   U(:,:,2,3,3)= 0;
   U(:,:,2,3,4)= 0;
   
   
   U(:,:,2,4,1)= 0;
   U(:,:,2,4,2)= 0;
   U(:,:,2,4,3)= 0;
   U(:,:,2,4,4)= 0;
   
   
  
   
   
   U(:,:,3,1,1)= Q*Q*T(:,:,1)*T(:,:,2)*T(:,:,3);
   U(:,:,3,1,2)= Q*T(:,:,1)*Q*T(:,:,2)*T(:,:,3);
   U(:,:,3,1,3)= Q*T(:,:,1)*T(:,:,2)*Q*T(:,:,3);
   U(:,:,3,1,4)= 0;
   
   
   U(:,:,3,2,1)= Q*T(:,:,1)*Q*T(:,:,2)*T(:,:,3);
   U(:,:,3,2,2)= T(:,:,1)*Q*Q*T(:,:,2)*T(:,:,3);
   U(:,:,3,2,3)= T(:,:,1)*Q*T(:,:,2)*Q*T(:,:,3);
   U(:,:,3,2,4)= 0;
   
   
   U(:,:,3,3,1)= Q*T(:,:,1)*T(:,:,2)*Q*T(:,:,3);
   U(:,:,3,3,2)= T(:,:,1)*Q*T(:,:,2)*Q*T(:,:,3);
   U(:,:,3,3,3)= T(:,:,1)*T(:,:,2)*Q*Q*T(:,:,3);
   U(:,:,3,3,4)= 0;
   
   
   U(:,:,3,4,1)= 0;
   U(:,:,3,4,2)= 0;
   U(:,:,3,4,3)= 0;
   U(:,:,3,4,4)= 0;
   
   
   U(:,:,4,1,1)= Q*Q*T(:,:,1)*T(:,:,2)*T(:,:,3)*T(:,:,4);
   U(:,:,4,1,2)= Q*T(:,:,1)*Q*T(:,:,2)*T(:,:,3)*T(:,:,4);
   U(:,:,4,1,3)= Q*T(:,:,1)*T(:,:,2)*Q*T(:,:,3)*T(:,:,4);
   U(:,:,4,1,4)= Q*T(:,:,1)*T(:,:,2)*T(:,:,3)*Q*T(:,:,4);
   
   U(:,:,4,2,1)= Q*T(:,:,1)*Q*T(:,:,2)*T(:,:,3)*T(:,:,4);
   U(:,:,4,2,2)= T(:,:,1)*Q*Q*T(:,:,2)*T(:,:,3)*T(:,:,4);
   U(:,:,4,2,3)= T(:,:,1)*Q*T(:,:,2)*Q*T(:,:,3)*T(:,:,4);
   U(:,:,4,2,4)= T(:,:,1)*Q*T(:,:,2)*T(:,:,3)*Q*T(:,:,4);
   
   
   U(:,:,4,3,1)= Q*T(:,:,1)*T(:,:,2)*Q*T(:,:,3)*T(:,:,4);
   U(:,:,4,3,2)= T(:,:,1)*Q*T(:,:,2)*Q*T(:,:,3)*T(:,:,4);
   U(:,:,4,3,3)= T(:,:,1)*T(:,:,2)*Q*Q*T(:,:,3)*T(:,:,4);
   U(:,:,4,3,4)= T(:,:,1)*T(:,:,2)*Q*T(:,:,3)*Q*T(:,:,4); 
   
   
   U(:,:,4,4,1)= Q*T(:,:,1)*T(:,:,2)*T(:,:,3)*Q*T(:,:,4);
   U(:,:,4,4,2)= T(:,:,1)*Q*T(:,:,2)*T(:,:,3)*Q*T(:,:,4);
   U(:,:,4,4,3)= T(:,:,1)*T(:,:,2)*Q*T(:,:,3)*Q*T(:,:,4);
   U(:,:,4,4,4)= T(:,:,1)*T(:,:,2)*T(:,:,3)*Q*Q*T(:,:,4);
   
   
   
   
C(1,1,1)= trace(U(:,:,1,1,1)*J(:,:,1)*transpose(F(:,:,1,1))) + trace(U(:,:,2,1,1)*J(:,:,2)*transpose(F(:,:,2,1))) + trace(U(:,:,3,1,1)*J(:,:,3)*transpose(F(:,:,3,1))) + trace(U(:,:,4,1,1)*J(:,:,4)*transpose(F(:,:,4,1)));
C(1,1,2)= trace(U(:,:,2,1,2)*J(:,:,2)*transpose(F(:,:,2,1))) + trace(U(:,:,3,1,2)*J(:,:,3)*transpose(F(:,:,3,1))) + trace(U(:,:,4,1,2)*J(:,:,4)*transpose(F(:,:,4,1))) ;
C(1,1,3)= trace(U(:,:,3,1,3)*J(:,:,3)*transpose(F(:,:,3,1))) + trace(U(:,:,4,1,3)*J(:,:,4)*transpose(F(:,:,4,1)));
C(1,1,4)= trace(U(:,:,4,1,4)*J(:,:,4)*transpose(F(:,:,4,1))) ;

C(1,2,1)= trace(U(:,:,2,2,1)*J(:,:,2)*transpose(F(:,:,2,1))) + trace(U(:,:,3,2,1)*J(:,:,3)*transpose(F(:,:,3,1))) + trace(U(:,:,4,2,1)*J(:,:,4)*transpose(F(:,:,4,1))) ;
C(1,2,2)= trace(U(:,:,2,2,2)*J(:,:,2)*transpose(F(:,:,2,1))) + trace(U(:,:,3,2,2)*J(:,:,3)*transpose(F(:,:,3,1))) + trace(U(:,:,4,2,2)*J(:,:,4)*transpose(F(:,:,4,1))) ; 
C(1,2,3)= trace(U(:,:,3,2,3)*J(:,:,3)*transpose(F(:,:,3,1))) + trace(U(:,:,4,2,3)*J(:,:,4)*transpose(F(:,:,4,1))) ;
C(1,2,4)= trace(U(:,:,4,2,4)*J(:,:,4)*transpose(F(:,:,4,1))) ;


C(1,3,1)= trace(U(:,:,3,3,1)*J(:,:,3)*transpose(F(:,:,3,1))) + trace(U(:,:,4,3,1)*J(:,:,4)*transpose(F(:,:,4,1)));
C(1,3,2)= trace(U(:,:,3,3,2)*J(:,:,3)*transpose(F(:,:,3,1))) + trace(U(:,:,4,3,2)*J(:,:,4)*transpose(F(:,:,4,1))) ;
C(1,3,3)= trace(U(:,:,3,3,3)*J(:,:,3)*transpose(F(:,:,3,1))) + trace(U(:,:,4,3,3)*J(:,:,4)*transpose(F(:,:,4,1))) ;
C(1,3,4)= trace(U(:,:,4,3,4)*J(:,:,4)*transpose(F(:,:,4,1))) ;


C(1,4,1)= trace(U(:,:,4,4,1)*J(:,:,4)*transpose(F(:,:,4,1))) ;
C(1,4,2)= trace(U(:,:,4,4,2)*J(:,:,4)*transpose(F(:,:,4,1))) ; 
C(1,4,3)= trace(U(:,:,4,4,3)*J(:,:,4)*transpose(F(:,:,4,1))) ;
C(1,4,4)= trace(U(:,:,4,4,4)*J(:,:,4)*transpose(F(:,:,4,1))) ;




C(2,1,1)= trace(U(:,:,2,1,1)*J(:,:,2)*transpose(F(:,:,2,2))) + trace(U(:,:,3,1,1)*J(:,:,3)*transpose(F(:,:,3,2))) + trace(U(:,:,4,1,1)*J(:,:,4)*transpose(F(:,:,4,2)));
C(2,1,2)= trace(U(:,:,2,1,2)*J(:,:,2)*transpose(F(:,:,2,2))) + trace(U(:,:,3,1,2)*J(:,:,3)*transpose(F(:,:,3,2))) + trace(U(:,:,4,1,2)*J(:,:,4)*transpose(F(:,:,4,2))) ;
C(2,1,3)= trace(U(:,:,3,1,3)*J(:,:,3)*transpose(F(:,:,3,2))) + trace(U(:,:,4,1,3)*J(:,:,4)*transpose(F(:,:,4,2))) ;
C(2,1,4)= trace(U(:,:,4,1,4)*J(:,:,4)*transpose(F(:,:,4,2))) ;


C(2,2,1)= trace(U(:,:,2,2,1)*J(:,:,2)*transpose(F(:,:,2,2))) + trace(U(:,:,3,2,1)*J(:,:,3)*transpose(F(:,:,3,2))) + trace(U(:,:,4,2,1)*J(:,:,4)*transpose(F(:,:,4,2)));
C(2,2,2)= trace(U(:,:,2,2,2)*J(:,:,2)*transpose(F(:,:,2,2))) + trace(U(:,:,3,2,2)*J(:,:,3)*transpose(F(:,:,3,2))) + trace(U(:,:,4,2,2)*J(:,:,4)*transpose(F(:,:,4,2))) ; 
C(2,2,3)= trace(U(:,:,3,2,3)*J(:,:,3)*transpose(F(:,:,3,2))) + trace(U(:,:,4,2,3)*J(:,:,4)*transpose(F(:,:,4,2))) ;
C(2,2,4)= trace(U(:,:,4,2,4)*J(:,:,4)*transpose(F(:,:,4,2)))  ;


C(2,3,1)= trace(U(:,:,3,3,1)*J(:,:,3)*transpose(F(:,:,3,2))) + trace(U(:,:,4,3,1)*J(:,:,4)*transpose(F(:,:,4,2))) ;
C(2,3,2)= trace(U(:,:,3,3,2)*J(:,:,3)*transpose(F(:,:,3,2))) + trace(U(:,:,4,3,2)*J(:,:,4)*transpose(F(:,:,4,2)))  ; 
C(2,3,3)= trace(U(:,:,3,3,3)*J(:,:,3)*transpose(F(:,:,3,2))) + trace(U(:,:,4,3,3)*J(:,:,4)*transpose(F(:,:,4,2)))  ;
C(2,3,4)= trace(U(:,:,4,3,4)*J(:,:,4)*transpose(F(:,:,4,2))) ;


C(2,4,1)= trace(U(:,:,4,4,1)*J(:,:,4)*transpose(F(:,:,4,2))) ;
C(2,4,2)= trace(U(:,:,4,4,2)*J(:,:,4)*transpose(F(:,:,4,2)))  ; 
C(2,4,3)= trace(U(:,:,4,4,3)*J(:,:,4)*transpose(F(:,:,4,2)))   ; 
C(2,4,4)= trace(U(:,:,4,4,4)*J(:,:,4)*transpose(F(:,:,4,2))) ; 




C(3,1,1)= trace(U(:,:,3,1,1)*J(:,:,3)*transpose(F(:,:,3,3))) + trace(U(:,:,4,1,1)*J(:,:,4)*transpose(F(:,:,4,3)))  ;
C(3,1,2)= trace(U(:,:,3,1,2)*J(:,:,3)*transpose(F(:,:,3,3))) + trace(U(:,:,4,1,2)*J(:,:,4)*transpose(F(:,:,4,3)))  ;
C(3,1,3)= trace(U(:,:,3,1,3)*J(:,:,3)*transpose(F(:,:,3,3))) + trace(U(:,:,4,1,3)*J(:,:,4)*transpose(F(:,:,4,3)))  ;
C(3,1,4)= trace(U(:,:,4,1,4)*J(:,:,4)*transpose(F(:,:,4,3)))  ;

C(3,2,1)= trace(U(:,:,3,2,1)*J(:,:,3)*transpose(F(:,:,3,3))) + trace(U(:,:,4,2,1)*J(:,:,4)*transpose(F(:,:,4,3)))  ;
C(3,2,2)= trace(U(:,:,3,2,2)*J(:,:,3)*transpose(F(:,:,3,3))) + trace(U(:,:,4,2,2)*J(:,:,4)*transpose(F(:,:,4,3)))  ;
C(3,2,3)= trace(U(:,:,3,2,3)*J(:,:,3)*transpose(F(:,:,3,3))) + trace(U(:,:,4,2,3)*J(:,:,4)*transpose(F(:,:,4,3)))  ;
C(3,2,4)= trace(U(:,:,4,2,4)*J(:,:,4)*transpose(F(:,:,4,3))) ;


C(3,3,1)= trace(U(:,:,3,3,1)*J(:,:,3)*transpose(F(:,:,3,3))) + trace(U(:,:,4,3,1)*J(:,:,4)*transpose(F(:,:,4,3))) ;
C(3,3,2)= trace(U(:,:,3,3,2)*J(:,:,3)*transpose(F(:,:,3,3))) + trace(U(:,:,4,3,2)*J(:,:,4)*transpose(F(:,:,4,3)))  ;
C(3,3,3)= trace(U(:,:,3,3,3)*J(:,:,3)*transpose(F(:,:,3,3))) + trace(U(:,:,4,3,3)*J(:,:,4)*transpose(F(:,:,4,3)))  ;
C(3,3,4)= trace(U(:,:,4,3,4)*J(:,:,4)*transpose(F(:,:,4,3)))  ;

C(3,4,1)= trace(U(:,:,4,4,1)*J(:,:,4)*transpose(F(:,:,4,3)))  ;
C(3,4,2)= trace(U(:,:,4,4,2)*J(:,:,4)*transpose(F(:,:,4,3)))  ; 
C(3,4,3)= trace(U(:,:,4,4,3)*J(:,:,4)*transpose(F(:,:,4,3)))  ;
C(3,4,4)= trace(U(:,:,4,4,4)*J(:,:,4)*transpose(F(:,:,4,3)))  ;





C(4,1,1)= trace(U(:,:,4,1,1)*J(:,:,4)*transpose(F(:,:,4,4)))  ;
C(4,1,2)= trace(U(:,:,4,1,2)*J(:,:,4)*transpose(F(:,:,4,4)))  ; 
C(4,1,3)= trace(U(:,:,4,1,3)*J(:,:,4)*transpose(F(:,:,4,4)))  ;
C(4,1,4)= trace(U(:,:,4,1,4)*J(:,:,4)*transpose(F(:,:,4,4)))   ;

C(4,2,1)= trace(U(:,:,4,2,1)*J(:,:,4)*transpose(F(:,:,4,4))) ;
C(4,2,2)= trace(U(:,:,4,2,2)*J(:,:,4)*transpose(F(:,:,4,4)))  ; 
C(4,2,3)= trace(U(:,:,4,2,3)*J(:,:,4)*transpose(F(:,:,4,4)))  ;
C(4,2,4)= trace(U(:,:,4,2,4)*J(:,:,4)*transpose(F(:,:,4,4)))  ;

C(4,3,1)= trace(U(:,:,4,3,1)*J(:,:,4)*transpose(F(:,:,4,4)))  ;
C(4,3,2)= trace(U(:,:,4,3,2)*J(:,:,4)*transpose(F(:,:,4,4)))  ; 
C(4,3,3)= trace(U(:,:,4,3,3)*J(:,:,4)*transpose(F(:,:,4,4)))  ;
C(4,3,4)= trace(U(:,:,4,3,4)*J(:,:,4)*transpose(F(:,:,4,4)))  ;

C(4,4,1)= trace(U(:,:,4,4,1)*J(:,:,4)*transpose(F(:,:,4,4))) ;
C(4,4,2)= trace(U(:,:,4,4,2)*J(:,:,4)*transpose(F(:,:,4,4)))  ; 
C(4,4,3)= trace(U(:,:,4,4,3)*J(:,:,4)*transpose(F(:,:,4,4)))  ;
C(4,4,4)= trace(U(:,:,4,4,4)*J(:,:,4)*transpose(F(:,:,4,4)))  ;





Co(1)= C(1,1,1)*qd(1)*qd(1)+C(1,2,2)*qd(2)*qd(2)+C(1,3,3)*qd(3)*qd(3)+C(1,4,4)*qd(4)*qd(4)+2*(C(1,1,2)*qd(1)*qd(2)+ C(1,1,3)*qd(1)*qd(3)+ C(1,1,4)*qd(1)*qd(4)+ C(1,2,3)*qd(2)*qd(3)+ C(1,2,4)*qd(2)*qd(4)+ C(1,3,4)*qd(3)*qd(4));

Co(2)= C(2,1,1)*qd(1)*qd(1)+C(2,2,2)*qd(2)*qd(2)+C(2,3,3)*qd(3)*qd(3)+C(2,4,4)*qd(4)*qd(4)+2*(C(2,1,2)*qd(1)*qd(2)+ C(2,1,3)*qd(1)*qd(3)+ C(2,1,4)*qd(1)*qd(4)+  C(2,2,3)*qd(2)*qd(3)+ C(2,2,4)*qd(2)*qd(4)+ C(2,3,4)*qd(3)*qd(4));

Co(3)= C(3,1,1)*qd(1)*qd(1)+C(3,2,2)*qd(2)*qd(2)+C(3,3,3)*qd(3)*qd(3)+C(3,4,4)*qd(4)*qd(4)+2*(C(3,1,2)*qd(1)*qd(2)+ C(3,1,3)*qd(1)*qd(3)+ C(3,1,4)*qd(1)*qd(4)+ C(3,2,3)*qd(2)*qd(3)+ C(3,2,4)*qd(2)*qd(4)+ C(3,3,4)*qd(3)*qd(4));

Co(4)= C(4,1,1)*qd(1)*qd(1)+C(4,2,2)*qd(2)*qd(2)+C(4,3,3)*qd(3)*qd(3)+C(4,4,4)*qd(4)*qd(4)+2*(C(4,1,2)*qd(1)*qd(2)+ C(4,1,3)*qd(1)*qd(3)+ C(4,1,4)*qd(1)*qd(4)+  C(4,2,3)*qd(2)*qd(3)+ C(4,2,4)*qd(2)*qd(4)+ C(4,3,4)*qd(3)*qd(4));


    
    
    
    
    
    
    
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