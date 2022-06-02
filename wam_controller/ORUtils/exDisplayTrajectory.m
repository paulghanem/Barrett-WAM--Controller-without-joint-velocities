function [ ] = exDisplayTrajectory(robotid, TRAJ, time )
%ORDISPLAYTRAJECTORY Displays the trajectory in the OpenRave window
%   
    [nJnts nPoints] = size(TRAJ);

    for i = 1:nPoints
        orRobotSetDOFValues(robotid, TRAJ(:,i)', 0:nJnts-1)
        if nargin == 3
            pause(time/nPoints);
        end
    end

end

