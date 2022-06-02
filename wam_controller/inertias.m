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
            
            
            
     I = zeros(3,3,7);
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
    I(:,:,5) =[0.00005587 , 0.00000026,  0.00000000;
               0.00000026 , 0.00007817,  -0.00000083;
               0.00000000 , -0.00000083,  0.00006594];
    I(:,:,6) = [0.00093106,  0.00000148 , -0.00000201;
                0.00000148,  0.00049833 , -0.00022162;
               -0.00000201,  -0.00022162,  0.00057483];
    I(:,:,7) = [ 0.003167342,          0,         0;       % Approximate Inertia Tensor for Schunk Gripper
                              0, 0.001128942,         0;
                              0,          0, 0.00258007;];       