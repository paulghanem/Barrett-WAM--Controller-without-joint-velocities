% % Test circLineAngles.  Speed is about 10 microseconds on core-i7
% function test
% tic
% for i = 1 : 1e7
% %     [q, nsoln] = circLineAngles(1/sqrt(2), 1/sqrt(2), 1);
% %     [q, nsoln] = circLineAngles(1, 0, -1)
% %    [q, nsoln] = circLineAngles(0, 1, 1)
%     [q, nsoln] = circLineAngles(rand*10, rand*10, rand/10);
% end
% toc
% end
%
function [q,nsoln] = circLineAngles(A,B,C)
% Compute solution to, A*cos(q) + B*sin(q)+ C = 0.  The solutions are the
% intersections of a line with the unit circle.  Let y=cos(q) and x=sin(q).
% Then we want to solve the system:
%         Ax + By + C = 0
%           x^2 + y^2 = 1
% where x = cos(q) and y = sin(q).
%
%   Solution:
%      Eliminating x and solving for y yields two solutions:
%          x = [-A*C -/+ B * sqrt(A^2 + B^2 - C^2)] / (A^2 + B^2)
%          y = [-B*C +/- A * sqrt(A^2 + B^2 - C^2)] / (A^2 + B^2)
%
% Normalize coefficients: C is the distance from the line to the circle
sqrtABsqrd = sqrt(A^2 + B^2);
A = A/sqrtABsqrd;  B = B/sqrtABsqrd;  C = C/sqrtABsqrd;
absC = abs(C);
if absC > 1.0001  % Assume a solution if the line is within .0001 of the circle
    % machine epsilon of the circle.
    nsoln = 0;
    q = [];
elseif absC > 0.9999999999995  % used vpa(cos(1e-6)) to get this number
    % The two solutions are within 0.00006 degrees, so return one solution
    nsoln = 1;
    q = atan2(-B*C, -A*C);
else
    nsoln = 2;
    ABCsqrt = sqrt(1 - C^2);
    % Could save 4 multiplies below if it matters.
    sinq1 = (-B*C + A*ABCsqrt);
    cosq1 = (-A*C - B*ABCsqrt);
    sinq2 = (-B*C - A*ABCsqrt);
    cosq2 = (-A*C + B*ABCsqrt);
    q(1) = atan2(sinq1, cosq1);
    q(2) = atan2(sinq2, cosq2);
end
end