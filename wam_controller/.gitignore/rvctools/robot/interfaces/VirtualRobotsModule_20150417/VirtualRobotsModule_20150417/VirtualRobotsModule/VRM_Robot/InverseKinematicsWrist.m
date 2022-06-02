function [solutionList] = InverseKinematicsWrist( orientationMatrix)
% Calculates the two sets of solutions for obot wrist, when the orientation
% matrix is given as input


% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
% OUTPUT IN RADIANS, ROWS CORRESPOND TO A SINGLE SOLUTION
% !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

% Copy the orienation matrix into the variable Q
Q = orientationMatrix;

% Solution1
theta_1(1) = atan2( Q(2,3), Q(1,3));
theta_2(1) = atan2( sqrt(Q(1,3)^2 + Q(2,3)^2),Q(3,3));
theta_3(1) = atan2(-Q(3,2),Q(3,1));

% theta_1(2) = atan2(-Q(2,3),-Q(1,3));
% theta_2(2) = atan2(-sqrt(Q(1,3)^2 + Q(2,3)^2),Q(3,3));
% theta_3(2) = atan2(-Q(3,2),Q(3,1));

theta_1(2) =  theta_1(1) + pi;;
theta_2(2) = -theta_2(1);
theta_3(2) =  theta_3(1) + pi;;


solutionList = [ theta_1' theta_2' theta_3'];




end