%Returns closest solution and all valid solutions
function [valid closest solnSet] = wam7ik_w_joint_limits(A07goal, qNear)
valid = 1; % Assume we will get a valid result
solnSet=[];
q3= [0 0 1 0 0 0 0]';
%Runs Generic Inverse Kinematics solver
for delta_q3= -2.9:.1:2.9
    [soln flag] = wam7ik_meters(A07goal,delta_q3*q3);
    if( sum(flag > 10) == 0 ) % If none of the joints have a flag > 10 (Means no solution)
        solnSet = [soln,solnSet];
    end
end

% If there were no solutions at all
if(size(solnSet,2) == 0)
    %warning('No solutions were found');
    valid = 0;        % Set valid flag to zero
    closest = qNear;  % Set closest to the input angle
    return;
end

wam_theta_min = [-2.62,   -2.01, -2.97, -.87, -4.79, -1.57, -3.0]';
wam_theta_max = [ 2.62,    2.01,  2.97, 3.14,  1.27,  1.57,  3.0]';

%Finds and deletes solutions that are out of range
s = size(solnSet,2);

inrange = (wam_theta_min*ones(1,s) > solnSet)| (wam_theta_max*ones(1,s) < solnSet);
invalid_cols =  find(sum(inrange,1));
solnSet(:,invalid_cols)=[];

% If no solution after removing joint limits
if(size(solnSet,2) == 0)
    %warning('No solutions were found that do not violate joint limits');
    valid = 0;        % Set valid flag to zero
    closest = qNear;  % Set closest to the input angle
    return;
end

%Remove duplicates
first_num=1;
second_num=2;
while(first_num < size(solnSet,2))
    while(second_num <= size(solnSet,2))
        if(abs(norm(solnSet(:,first_num)) - norm(solnSet(:,second_num))) <1e-5 )
            solnSet(:,second_num)=[];
        else
            second_num=second_num+1;
        end
    end
    second_num= first_num+2;
    first_num=first_num+1;
end

%Finds closest solution
for i =1:size(solnSet,2)
    normal(i) = norm(solnSet(:,i) - qNear);
end

[~,I]=min(normal);
closest = solnSet(:,I);