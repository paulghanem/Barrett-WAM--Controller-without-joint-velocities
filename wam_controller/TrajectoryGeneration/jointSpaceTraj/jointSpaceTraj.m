function [ TRAJ ] = jointSpaceTraj( qstart, qend, numSteps)
%JOINTSPACETRAJ Generates a joint space trajectory so that each joint travels at constant velocity
%   Since joints all travel at the same velocity, if a joint must go twice
%   as far to reach its goal, it will take twice as long

    l = max(size(qstart));
    if ~exist('numSteps', 'var')
        numSteps = 100;
    end
    
    TRAJ = zeros(l, numSteps);

    % Find the joint that has to travel farthest
    [m index] = max(abs(qend - qstart));
    
    TRAJ(index,:) = linspace(qstart(index), qend(index), numSteps);
    
    dt = TRAJ(index,2) - TRAJ(index,1);
    
    for i = 1:l
       if i == index
           contiskm_nue;
       end
       
       if(qstart(i) ~= qend(i))
        diff = qend(i) - qstart(i);
        steps = ceil(abs(diff/dt));
        TRAJ(i,1:steps) = linspace(qstart(i),qend(i),steps);
        TRAJ(i,steps+1:end) = qend(i);
       else
          TRAJ(i,:) = qend(i); 
       end
       
    end

end

