function [ T ] = getTTrayObj( object )
%EXGETTTRAYOBJ Summary of this function goes here
%   Detailed explanation goes here

    switch(object)
        case '5.03' % HAT
            T = [ 1  0  0  0.075; % Actual X
                  0  1  0  0.075; % Actual Y
                  0  0  1  0.145; % Actual Z
                  
                   %1  0  0  0.07;
                   %0  1  0  0.1;
                   %0  0  1  0.1;     
                  
%                   1  0  0  0.105;  % Diagonal offset to drive cause reaction
%                   0  1  0  0.035;
%                   0  0  1  .115;

                  0  0  0     1];
            
        case '5.04' % Tumbler
            T = [ 0  1  0  0.047;
                 -1  0  0  0.047;
                  0  0  1  0.09;
                  0  0  0    1];
              
        case '5.01' % Cinder Block
            T = [ 1 0 0 .0625; % Actual X
                  0 1 0 .1275; % Actual Y
                  0 0 1 .05; % Actual Z
                  0 0 0 1;];
              
        case '5.02' % Brick
            T = [ 1 0 0 .0625;
                 0 1 0 .1025;
                 0 0 1 .024;
                 0 0 0 1];
             
        case '2.01' % Block
            % Rotated block
            T = [0  -1  0  0.015;
                 1   0  0  0.46;
                 0   0  1  0.075;
                 0   0  0  1];
             
%             T = [1   0  0  0.06;
%                  0   1  0  0.055;
%                  0   0  1  0.12;
%                  0   0  0  1];

        
% %             % For rabbit[
%             T = [ 1 0 0 .0825;
%                   0 1 0 .0825;
%                   0 0 1 .024;
%                   0 0 0 1];
            
        otherwise
            TTrayCylinder = [1  0  0  0.03175;
                             0  1  0    0.127;
                             0  0  1  0.00635+0.0127+.008;
                             0  0  0        1];


            TTrayUChannel = [ 0  1  0  0.0053975;
                             -1  0  0    0.00635;
                              0  0  1     0.0508+.020;
                              0  0  0          1];
            T = eye(4);
    end
end
