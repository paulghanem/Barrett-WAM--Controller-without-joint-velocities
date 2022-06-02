%Returns closest solution and all valid solutions
function [closest solnSet] = wam7ik_w_joint_limits_new(A07goal, qNear)
solnSet=[];
q3= [0 0 1 0 0 0 0]';
%Runs Generic Inverse Kinematics solver
for delta_q3= -2.9:.1:2.9
    
solnSet = [wam7ik_meters_new(A07goal,delta_q3*q3),solnSet];
end

wam_theta_min = [-1.5708, -1.0472, -2.97, -.87, -4.79, -1.57, -3.0]';
wam_theta_max = [ 1.5708,    2.01,  2.97, 3.14,  1.27,  1.57,  3.0]';

%Finds and deletes solutions that are out of range
s= size(solnSet,2);
inrange = (wam_theta_min*ones(1,s) > solnSet)| (wam_theta_max*ones(1,s) < solnSet);
invalid_cols =  find(sum(inrange,1));
solnSet(:,invalid_cols)=[];
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
if(size(solnSet,2)==0)
    closest = qNear;
    return;
end


%Finds closest solution
for i =1:size(solnSet,2)
    normal(i) = norm(solnSet(:,i) - qNear);
end
[~,I]=min(normal);
closest = solnSet(:,I);