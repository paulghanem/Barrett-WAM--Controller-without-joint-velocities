
function [valid q] = generateTrajectory(q_start,T_goal,iterations,q3_samples,wamInfo)
%q_start is 7X1 joint angles
%T-goal is 3X4 target transformation
%iteration is number of time samples
%q3_samples is how sparsely q3 is sampled. decrease for speed

    %iterations = 20;

    wam_theta_min = [-1.5708,   -2.01, -2.97, -.87, -4.79, -1.57, -3.0];
    wam_theta_max = [ 1.5708,    2.01,  2.97, 3.14,  1.27,  1.57,  3.0];

    qset=struct();

    % calculate task space trajectory
    T_start = wam7fk_meters(q_start,wamInfo);
    Traj = ctraj(T_start,T_goal, iterations);

    % find all posible starting joint configurations
    [valid, q(:,1) qset(1).soln]= wam7ik_w_joint_limits_new(T_start,q_start);

    if valid
        % find set of joint angles at all configurations
        for i = 2:iterations
            [~, q(:,i) qset(i).soln]= wam7ik_w_joint_limits_new(Traj(:,:,i),q(:,i-1)); 
        end

        %find minimal path through solution tree
        qloc = wam7Tree(qset,iterations);

        %set appropiate output angles
        for i = 1:iterations
            q(:,i)=qset(i).soln(:,qloc(i));
            if(norm(q(:,i))==0)

            end
        end
    else
        valid = 0;
        q = q_start;
    end
end



