function [ T ] = exGetTTrayObj( object )
%EXGETTTRAYOBJ Summary of this function goes here
%   Detailed explanation goes here
    
    switch(object)
        case '7.01' % HAT
            T = [1  0  0  0.042;
                 0  1  0  0.042;
                 0  0  1  0.105;
                 0  0  0    1];
            
        case '8.01' % Tumbler
            T = [ 0  1  0  0.042;
                 -1  0  0  0.042;
                  0  0  1  0.105;
                  0  0  0    1];
              
        case 'cube'
            T = [%1   0  0  0.01;
                 1  0  0  0.0381;
                %0  1  0  0.0127;
                 0  1  0  .03;
                 0  0  1  0.0508;
                 0  0  0       1];

        otherwise
            TTrayCylinder = [1  0  0  0.03175;
                             0  1  0    0.127;
                             0  0  1  0.00635+0.0127+.008;
                             0  0  0        1];

            TTrayCube = [1   0  0  0.01;
                         %1  0  0  0.0381;
                         %0  1  0  0.0127;
                         0  1  0  .03;
                         0  0  1  0.0508;
                         0  0  0       1];

            TTrayUChannel = [ 0  1  0  0.0053975;
                             -1  0  0    0.00635;
                              0  0  1     0.0508+.020;
                              0  0  0          1];
            T = eye(4);      
    end
end
