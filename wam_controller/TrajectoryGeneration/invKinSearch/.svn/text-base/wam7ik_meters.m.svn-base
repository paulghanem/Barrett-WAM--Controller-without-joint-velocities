function [soln, flag] = wam7ik_meters(A07goal, qNear)
% Inv Kin of Barrett 7-dof WAM.
%   Given a homogeneous transform, A07_goal, describing the transform of
% the end effector with respect to the WAM base, compute the inverse
% kinematic solutions, q = (q1,q2,q3,q4,q5,q6,q7), that achieve A07_goal.
% Generally, when one solution exists, an infinite number do.
%
% We'd like to identify the set of all q such that:
%                  A07(q) = A07_goal
% where A07_goal is a homogeqneous transform of known numbers (r11, r12,
% r13, p1, r21, r22, r23, p2, r31, r32, r33, p3).
%
% Because the wrist is spherical, this problem can be partitioned into an
% arm inverse kinematics problem and a wrist inverse kinematic problem.
%
% For generic configurations there are 8 solutions corresponding to elbow
% up or down, wrist flip left or right, and base flip left or right.
%
%  Output:
%      soln - a matrix of solutions, q vectors in columns
%      flag - return information about solution special cases
%
% by Jeff Trinkle
% Feb 14, 2012

%==========================================================================
%%%%%%%% SET UP INV KIN SOLN AND CHECK SIMPLE EXISTENCE CONDITIONS %%%%%%%%
%--------------------------------------------------------------------------
% Debug level
debug = 0;  % debug = 1 yields some debugging info
            % debug = 2 yields more...

% The values of non-zero d's, a's, and alpha's.  The their values in meters
% are:
a3 = 0.045;  d3 = 0.55;  d5 = 0.3;
%d7 = 0.0161; % Nothing attached
d7 = .1563;   % Schunk Gripper attached
A07goal = A07goal/transl(0,0,d7);
% a3 = 45;   d3 = 550;   d5 = 300;  % d7 = 60;   millimeters
% I removed transl 0.06 in A67 along z to put origin of frame 7 at the
% wrist's center.  You need a constant transform to take you from the wrist
% center out to where ever you want the tool frame.

% Assume the problem is generic.
soln = zeros(7,8);

% Identify special solution cases for each joint, so you can figure out how
% to navigate away from or through singular configurations.
flag = zeros(7,1); % flag(i) > 10   means no solution exists for that joint  

% qNear is a vector of joint angles that you'd like the returned solutions
% to be near.  Currently all solutions are returned, but this information
% could be used to return only the closest solution.
%     Start with the assumption that qNear(3) is possible for both q4s
% that will be found.  This might not be true.  In that case, we will
% adjust q3 to the nearest velue for which both q4s are feasible.
q3goal = qNear(3);
c3 = cos(q3goal);  s3 = sin(q3goal);

% Check the configuration for solution existence and singularity with
% respect to the elbow joint, q4.
pGoal = A07goal(1:3,4);     % vector from base origin to wrist origin
dist07goal = norm(pGoal);   % dist from base origin to target wrist origin

% The maximum distance from the base origin to the wrist origin is:
%    0.8551940647m    when q4 = 0.230526289738567286 radians
dist07max = sqrt(a3^2 + d3^2) + sqrt(a3^2 + d5^2);
%dist07max = 0.85519406474203130131;    % meters
% dist07max = 855.19406474203130131;    % millimeters
 
% Minimum distance is 0.2484816123m  when q4 = -2.9110663638512258578
dist07min = sqrt(a3^2 + d3^2) - sqrt(a3^2 + d5^2); 
%dist07min = 0.24848161225733078616;    % meters
% dist07min = 248.48161225733078616;    % millimeters

% Existence test for q4.
tol = 10*eps;  % This tolerance should be set based on numerical analysis
if dist07goal > dist07max + tol
    % Outside workspace too far from base - NO q4 EXISTS
    flag(4) = 11;
    if debug >= 1
        fprintf('Goal position of wrist origin beyond reach.  flag(4) = %2d\n', flag);
    end
    return;
elseif dist07goal < dist07min - tol
    % Outside workspace too near base - NO q4 EXISTS
    flag(4) = 12;
    if debug >= 1
        fprintf('Goal position of wrist origin too close to base.  flag(4) = %2d\n', flag);
    end
    return;
end

%==========================================================================
%%%%%%%%%% q4 EXISTS.  FIND TWO SOLUTIONS (GENERIC CASE) OR ONE. %%%%%%%%%%
%--------------------------------------------------------------------------
% Assume generic case with 8 solutions for a fixed q3 = q3goal.  Start
% assembling solution array.
% nsoln = [1; 2; 1; 2; 1; 2; 1];  % Generic soln multiplicities.  
soln(3,:) = q3goal*ones(1,8);   % Assume all solns have the goal q3 = qNear(3).

% Solve for q4 directly.  Expand the forward kinematic equation.  Square
% and add the three position elements defining the vector from the origin
% of the base frame {0} to the origin of the wrist frame {7}.
% (see wam7symbKin.m):
%
%  A*cos(q4) + B*sin(q4) + C = 0                     *(1)*
%
% where
% A = 2*(a3*a3 - d3*d5);
% B = -2*a3*(d3 + d5);
% C = p1^2 + p2^2 + p3^2 - (2*a3*a3 + d3*d3 + d5*d5);
%
% We know that:   sin(q4)^2 + cos(q4)^2 = 1          *(2)*
%
% The solution(s) are returned by [q4, nsoln] = circLineAngles(A,B,C); but
% using atan2 functions is about 10 times faster.

A = 2*(d3*d5 - a3^2);
B = 2*a3*(d3 + d5);
C = 2*a3*a3 + d3*d3 + d5*d5 - pGoal(1)^2 - pGoal(2)^2 - pGoal(3)^2;
[q4c, nsoln4] = circLineAngles(A,B,C);  % Slower soln, but check accuracy.
q4 = atan2(B,A);   % Use atan2.  It's faster.
%q4c
%q4
if dist07goal < dist07min + tol
    % Wrist center is inner bounding sphere of workspace.  One soln for q4.
   soln(4,:) = [q4*ones(1,4)    q4*ones(1,4)]; 
    %{
    if nsoln4 == 2
        flag(4) = 1;
        q4diff = abs(q4c(1) - q4c(2));
        q4err = abs(((q4c(1) + q4c(2)) / 2) - q4);
        fprintf('q4 should be unique, but circLineAngle found two q4s. flag(4) = %2d\n', flag(4));
        if debug >= 1
            if q4diff > 1e-8
                fprintf('q4s found by circLineAngle differ by %12.6g\n', q4diff);
            end
            if q4err > 1e-8
                fprintf('q4 found by circLineAngle not same as atan2 result. q4err = %12.6g\n', q4err);
            end
        end
        soln(4,:) = [q4(1)*ones(1,4)    q4(2)*ones(1,4)];
    elseif nsoln4 == 1
        % q4 = [q4  q4];
        % Return 8 solutions even if some are duplicates, to make future
        % processing easier.
        soln(4,:) = [q4(1)*ones(1,4)    q4(1)*ones(1,4)]; 
    elseif nsoln4 == 0
        flag(4) = 2;
        if debug >= 1
            fprintf('There should be one soln for q4, not zero.  flag(4) = %2d\n', flag(4));
        end
        return;
end
    %}
elseif dist07goal > dist07max - tol
    % On outer bounding sphere of workspace  One soln for q4.
    if nsoln4 == 2
        flag(4) = 3;
        q4diff = abs(q4c(1) - q4c(2));
        q4err = abs(((q4c(1) + q4c(2)) / 2) - q4);
        fprintf('q4 should be unique, but circLineAngle found two q4s. flag(4) = %2d\n', flag(4));
        if debug >= 1
            if q4diff > 1e-8
                fprintf('q4s found by circLineAngle differ by %12.6g\n', q4diff);
            end
            if q4err > 1e-8
                fprintf('q4 found by circLineAngle not same as atan2 result. q4err = %12.6g\n', q4err);
            end
        end
        soln(4,:) = [q4(1)*ones(1,4)    q4(2)*ones(1,4)];
    elseif nsoln4 == 1
        % q4 = [q4  q4];
        % Return 8 solutions even if some are duplicates, to make future
        % processing easier.
        soln(4,:) = [q4(1)*ones(1,4)    q4(1)*ones(1,4)]; 
    elseif nsoln4 == 0
        flag(4) = 4;
        if debug >= 1
            fprintf('There should be one soln for q4, not zero.  flag(4) = %2d\n', flag(4));
        end
        return;
    end
else
    % In the interior of the workspace.  Two solutions exist for q4.
    % Solve for q4
    [q4, nsoln4] = circLineAngles(A,B,C);  % Replace with faster solution.
    if nsoln4 == 2
        % Generic case - two solutions.
        soln(4,:) = [q4(1)*ones(1,4)    q4(2)*ones(1,4)];
    elseif nsoln4 == 1
        q4 = [q4  q4];   % This makes future processing easier.
        soln(4,:) = [q4(1)*ones(1,4)    q4(2)*ones(1,4)];
        flag(4) = 5;
        if debug >= 1
            fprintf('There should be two solns for q4, not one.  flag(4) = %2d\n', flag(4));
        end
    elseif nsoln4 == 0
        flag(4) = 6;
        if debug >= 1
            fprintf('There should be one soln for q4, not zero.  flag(4) = %2d\n', flag(4));
        end
        return;
    end
end

if debug >= 2
    for i = 1:8
        % Would be nice to vectorize wam7fk, so this loop was not necessary
        A07tmp = wam7fk_meters(soln(:,i));  
        errDist = norm(pGoal) - norm(A07tmp(1:3,4));
        if errDist > 1e-12
            fprintf('Error after solving for q4.  errDist = %8.4g\n', errDist);
        end
    end
end


%==========================================================================
%%%%%%%%%%%%%%%%%%%%%%%%%% FIND SOLUTIONS FOR q2. %%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------------------------------------------------------------
% Choose q2 to put the wrist origin at the proper height, p3.
%
%           A*cos(q2) + B*sin(q2) = C
% where
% A = (d3 + c4*d5 + a3*s4)
% B = c3*(a3*c4 - a3 - d5*s4)
% C = p(3)
%
% Perhaps a better approach to solving this equation is from Appendix C in
% Craig's text:
%    
%          q2 = atan2(B,A) +/- atan2(sqrt(A^2+B^2-C^2),C)
% 
% Craig's book seems to have an error.  I am using atan2(-B,-A) and getting
% correct results.  
for i = 1 : 2
    % When q3 ~= 0, it is possible to have a solution for q2 (generically,
    % two solutions) for one q4 and zero solutions for the other q4.
    % Because of this, always find the range of q3 for which the two q4s
    % will work.  This range is symmetric about 0 and pi.
    c4 = cos(soln(4,i*4));   s4 = sin(soln(4,i*4));
    rb = a3*(1-c4) + d5*s4;
    x1b = rb * s3;
    y1b = rb * c3;
    z1b = d3 + a3*s4 + d5*c4;  
    zGoal = A07goal(3,4);
    z1bmax = sqrt(z1b^2 + rb^2);
    
    % This test will be redundant when the code is completed.
    % This test will fail for at most one pass trough this loop.
    A = (d3 + c4*d5 + a3*s4);
    B = c3 * (a3*c4 - a3 - d5*s4);
    C = -pGoal(3);
    test = A^2 + B^2 - C^2;
    
    if abs(z1b) >= abs(zGoal)   % A^2 + B^2 - C^2 >= 0 for sure
        % No restrictions on q2.  Even in the worst case,
        % (q3 = +/-pi/2) the max height of the wrist center is higher
        % than the desired height.
        % The second q4 solution should always enter this portion of the
        % code.
        % This is a generic case.  Compute the values of q2 for the current
        % q3 and q4.
        alpha = atan2(-B,-A);
        beta = atan2(sqrt(A^2+B^2-C^2),C);
        q21 = alpha + beta;
        q22 = alpha - beta;
        % Put solution into (-pi,pi];
        q21 = mod(q21+pi, 2*pi) - pi;
        q22 = mod(q22+pi, 2*pi) - pi;
        soln(2, 4*i-3: 4*i) = [q21*ones(1,2)    q22*ones(1,2)];
    elseif abs(z1b) < abs(zGoal) % && z1bmax > abs(zGoal)
        % There is a range of q3s for which the current q4 can yield a
        % solution. Find the edges of that range.  The q3goal could be
        % inside or outside of this range.
        c3maxSqrd = (zGoal^2 - z1b^2) / rb^2;
        c3max = sqrt(c3maxSqrd);
        s3max = sqrt(1 - c3maxSqrd);
        q3max = atan2(s3max,c3max);  % q3max always in [0, pi/2]
      
        % Check to see if current q3 is in the feasible set
        if abs(pi/2 - abs(q3goal)) < pi/2 - q3max
            % q3 not in feasible set, so move to nearest boundary
            flag(3) = 1;
            if debug >= 1
                fprintf('q3 not in the feasible set for current q2.  flag(3) = %2d\n', flag(3));
            end
            % Move q3 to nearest boundary of the feasible region.
            if q3goal > pi/2
                q3new = pi - q3max;
            elseif q3goal > 0
                q3new = q3max;
            elseif q3goal > -pi/2
                q3new = -q3max;
            else
                q3new = q3max - pi;
            end
            soln(3, 4*i-3: 4*i) = [q3new*ones(1,4)];
            c3new = cos(q3new);
            Bnew = c3new * (a3*c4 - a3 - d5*s4);
            [q2, nsoln2] = circLineAngles(A,Bnew,C);
            alpha = atan2(-Bnew/C,-A/C);
            if nsoln2 == 2
                % This could happen from numerical error mismatch.
                flag(2) = 1;
                if debug >= 1
                    fprintf('There should be one soln for q2, not two.  flag(2) = %2d\n', flag(2));
                end
            elseif nsoln2 == 1
                % Generic situation
                soln(2, 4*i-3: 4*i) = [q2*ones(1,4)];
            elseif nsoln2 == 0 
                % This should not happen.
                flag(2) = 2;
                if debug >= 1
                    fprintf('There should be one soln for q2, not zero.  flag(2) = %2d\n', flag(2));
                end
                return;
            end
        else
            % Really the conde should never enter this section.  When
            [q2, nsoln2] = circLineAngles(A,B,C);
            alpha = atan2(-B,-A);
            beta = atan2(sqrt(A^2+B^2-C^2),C);
            q21 = alpha + beta;
            q22 = alpha - beta;
            % Put solution into (-pi,pi];
            q21 = mod(q21+pi, 2*pi) - pi;
            q22 = mod(q22+pi, 2*pi) - pi;
            if nsoln2 == 2
                % Generic situation.  Should be two solutions
                soln(2, 4*i-3: 4*i-2) = [q2(1)*ones(1,2)];
                soln(2, 4*i-1: 4*i) = [q2(2)*ones(1,2)];
            elseif nsoln2 == 1
                % Nongeneric.  Must have q3 on bndry of feasible region.
                flag(2) = 3;
                if debug >= 1
                    fprintf('q3goal must be right on the edge of the feasible region.  flag(2) = %2d\n', flag(2));
                end
                soln(2, 4*i-3: 4*i) = [q2(1)*ones(1,4)];  % Duplicate solution
            elseif nsoln2 == 0
                % This should not happen.
                flag(2) = 4;
                if debug >= 1
                    fprintf('q3goal must be outside the feasible region.  Earlier computations must be wrong.  flag(2) = %2d\n', flag(4));
                end
            end
        end
    end
end

if debug >= 2
    for i = 1:4
        A07tmp = wam7fk_meters(soln(:,i*2));
        errZ = norm(pGoal(3) - A07tmp(3,4));
        errXY = norm(A07goal(1:2,4)) - norm(A07tmp(1:2,4));
        if errZ > 1e-12 || errXY > 1e-12
            fprintf('Error after solving for q2.\n');
            fprintf('errZ = %8.4g     errXY = %8.4g\n', errZ, errXY);
        end
    end
end

%==========================================================================
%%%%%%%%%%%%%%%%%%%%%%%%%% FIND SOLUTIONS FOR q1. %%%%%%%%%%%%%%%%%%%%%%%%%
%--------------------------------------------------------------------------
% Prepare to solve for q1.  From symbolic representation of p(1) and p(2)
%
%           A*cos(q1) + B*sin(q1) = p1
%          -B*cos(q1) + A*sin(q1) = p2
% where
% A = d3*s2 + d5*(c4*s2 + c2*c3*s4) + a3*c2*c3 + a3*s2*s4 - a3*c2*c3*c4;
% B = a3*c4*s3 - a3*s3 - d5*s3*s4;
%
% In the generic case, this has a unique solution for each (q2,q3,q4): 
%               q1 = atan2(B*p(1)+A*p(2), A*p(1)-B*p(2))
% see Appendix C of Craig's book, ed. 2.
p_on_z0 = norm(pGoal(1:2));
if p_on_z0 <= tol
    % Can rotate the base without moving the origin of the wrist.  Any q1
    % will work, therefore set all solutions to the target q1.
    flag(1) = 1;
    soln(1,:) = qNear(1) * ones(1,8);
    if debug >= 1
        fprintf('The goal wrist origin is on the z0 axis.  flag = %2d\n', flag);
        fprintf('Any value of q1 is possible.  Set q1 to qNear(1).');
    end
else
    for i = 1 : 4
        % Generic situation - unique solution for q1
        c2 = cos(soln(2, 2*i));   s2 = sin(soln(2, 2*i));
        c3 = cos(soln(3, 2*i));   s3 = sin(soln(3, 2*i));
        c4 = cos(soln(4, 2*i));   s4 = sin(soln(4, 2*i));
        A = d3*s2 + d5*(c4*s2 + c2*c3*s4) + a3*c2*c3 + a3*s2*s4 - a3*c2*c3*c4;
        B = a3*c4*s3 - a3*s3 - d5*s3*s4;
        % [q1x, nsoln1] = circLineAngles(A,B,-p(1));
        % [q1y, nsoln1] = circLineAngles(-B,A,-p(2));
        q1 = atan2(B*pGoal(1)+A*pGoal(2), A*pGoal(1)-B*pGoal(2));
        soln(1, 2*i-1: 2*i) = [q1*ones(1,2)];
    end
end

if debug >= 2
    for i = 1:4
        A07tmp = wam7fk_meters(soln(:,2*i));
        err = norm(A07goal(1:3,4) - A07tmp(1:3,4));
        if err > 1e-12
            fprintf('Error after solving for q1.  err = errZ = %12.6g\n', err);
        end
    end
end


%==========================================================================
%%%%%%%%%%%%%%%%%%%%% FIND SOLUTIONS FOR q5, q6, q7. %%%%%%%%%%%%%%%%%%%%%%
%--------------------------------------------------------------------------
% Finally, solve for the last three angles - the spherical wrist joints.
% A47 = inv(A04) * A07_goal = a matrix of 12 known values, since q1, q2,
% q3, and q4 have been found already.
for i = 1 : 4
    q = soln(:, 2*i);   % solve for each pair of (q2, q4) possibilities.
    % From wam7symbKin.m    
% [ - c4*(s1*s3 - c1*c2*c3) - c1*s2*s4, - c3*s1 - c1*c2*s3, c1*c4*s2 - s4*(s1*s3 - c1*c2*c3)]
% [   c4*(c1*s3 + c2*c3*s1) - s1*s2*s4,   c1*c3 - c2*s1*s3, s4*(c1*s3 + c2*c3*s1) + c4*s1*s2]
% [                 - c2*s4 - c3*c4*s2,              s2*s3,                 c2*c4 - c3*s2*s4]
% 
% [a3*c4*(s1*s3 - c1*c2*c3) + c1*d3*s2 - a3*s1*s3 + a3*c1*s2*s4 + a3*c1*c2*c3]
% [a3*c1*s3 - a3*c4*(c1*s3 + c2*c3*s1) + d3*s1*s2 + a3*s1*s2*s4 + a3*c2*c3*s1]
% [c2*d3 - a3*c3*s2 + a3*c2*s4 + a3*c3*c4*s2]
    c1 = cos(q(1));      s1 = sin(q(1));
    c2 = cos(q(2));      s2 = sin(q(2));
    c3 = cos(q(3));      s3 = sin(q(3));
    c4 = cos(q(4));      s4 = sin(q(4));

    R04 = [-c4*(s1*s3 - c1*c2*c3) - c1*s2*s4, - c3*s1 - c1*c2*s3, c1*c4*s2 - s4*(s1*s3 - c1*c2*c3);
            c4*(c1*s3 + c2*c3*s1) - s1*s2*s4,   c1*c3 - c2*s1*s3, s4*(c1*s3 + c2*c3*s1) + c4*s1*s2;
                - c2*s4 - c3*c4*s2,                 s2*s3,                    c2*c4 - c3*s2*s4];

    p04 = [a3*c4*(s1*s3 - c1*c2*c3) + c1*d3*s2 - a3*s1*s3 + a3*c1*s2*s4 + a3*c1*c2*c3;
           a3*c1*s3 - a3*c4*(c1*s3 + c2*c3*s1) + d3*s1*s2 + a3*s1*s2*s4 + a3*c2*c3*s1;
           c2*d3 - a3*c3*s2 + a3*c2*s4 + a3*c3*c4*s2];
   
    A04 =    [R04      p04;     0 0 0 1];
    A04inv = [R04'  -R04'*p04;  0 0 0 1];

   
    % These next few commands are slow.
%     A01 = trotz(q(1))                   * trotx(-pi/2);
%     A12 = trotz(q(2))                   * trotx(pi/2);
%     A23 = trotz(q(3)) * transl(a3,0,d3) * trotx(-pi/2);
%     A34 = trotz(q(4)) * transl(-a3,0,0) * trotx(pi/2);
%     A04old = A01 * A12 * A23 * A34;

%     A47goal_old = A04old \ A07goal;  % Of course the inv can be closed form
    A47goal = A04inv * A07goal;
    
    % Compare the rotation matrix of A47 to (inv(A04) * A07_goal)
    % r11 = cos(q5)*cos(q6)*cos(q7) - sin(q5)*sin(q7);
    % r12 = -cos(q7)*sin(q5) - cos(q5)*cos(q6)*sin(q7);
    % r13 = cos(q5)*sin(q6);
    % r21 = cos(q5)*sin(q7) + cos(q6)*cos(q7)*sin(q5);
    % r22 = cos(q5)*cos(q7) - cos(q6)*sin(q5)*sin(q7);
    % r23 = sin(q5)*sin(q6);
    % r31 = -cos(q7)*sin(q6);
    % r32 = sin(q6)*sin(q7);
    % r33 = cos(q6);
    r = A47goal(1:3,1:3);
    
    %=====================
    % Solve for q6
    %---------------------
    %    sin(q6) = +/- sqrt(r13^2 + r23^2)
    %    cos(q6) = r33
    if r(3,3) > 1 - tol
        % Singular case.  Axes 4 and 6 align.
        % q6 is unique, but only the sum of q5 and q7 is unique
        q6 = 0;
        q5p7 = atan2(r(2,1), r(1,1));  % q5 + q7 is a constant
        diff = qNear(7) - qNear(5);
        q5 = (q5p7 - diff) / 2;
        q7 = q5 + diff;
%         q5 = q5p7/2;
%         q7 = q5;
        flag(6) = 1;
        if(q5>1.27) q5 = q5-2*pi;end
        soln(5:7, 2*i-1 : 2*i) = [q5 q5; q6 q6; q7 q7];  % Just duplicate solution
        if debug >= 1
            fprintf('Wrist in singular configuration; q5 + q7 = constant.  flag(6) = %2d\n', flag(6));
        end
    elseif r(3,3) < -(1 - tol)  % Joint limits prevent this case.
        % Singular case.  Axes 4 and 6 anti-align.
        % q6 is unique, but only the difference of q5 and q7 is unique
        q6 = pi;
        q5m7 = atan2(-r(2,1), -r(1,1));  % q5 - q7 is a constant
        qave = (qNear(5) + qNear(7)) / 2;
        q5 = qave + q5m7/2;
        q7 = q5 - q5m7;
        flag(6) = 2;
        if(q5>1.27) q5 = q5-2*pi;end
        soln(5:7, 2*i-1 : 2*i) = [q5 q5; q6 q6; q7 q7];  % Just duplicate solution
        if debug >= 1
            fprintf('Wrist in singular configuration; q5 - q7 = constant.  flag(6) = %2d\n', flag(6));
        end
    else
        % The generic case.  Two solutions exist
        q61 = atan2(sqrt(r(1,3)^2 + r(2,3)^2), r(3,3));
        q62 = -q61;
        
        %    cos(q5) = r13 / sin(q6)
        %    sin(q5) = r23 / sin(q6)
        % The positive and negative values of sin(q6) gives two solutions
        % for q5 that are pi radians apart.
        q51 = atan2(r(2,3), r(1,3));
        q52 = atan2(-r(2,3), -r(1,3));
        
        %    cos(q7) = -r31 / sin(q6)
        %    sin(q7) = r32 / sin(q6)
        % Two solutions here for the same reason as above.
        q71 = atan2(r(3,2), -r(3,1));
        q72 = atan2(-r(3,2), r(3,1));
        if(q51>1.27) q51 = q51-2*pi;end % fixes q5 quadrant issue
        if(q52>1.27) q52 = q52-2*pi;end
        soln(5:7, 2*i-1 : 2*i) = [q51 q52; q61 q62; q71 q72];
    end
end
%=========================================================================%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% SOLUTION COMPLETE %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%-------------------------------------------------------------------------%
if debug >= 2
    for i = 1:8
        A07tmp = wam7fk_meters(soln(:,i));  % This function is slow
        err = norm(A07goal - A07tmp);
        if err > 1e-8
            fprintf('hTform err norm = %12.8g\n', err);
        end
    end
end
end