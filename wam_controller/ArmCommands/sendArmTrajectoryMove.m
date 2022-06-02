function sendArmTrajectoryMove( netInfo, trajectory, duration )
%SENDARMTRAJECTORYMOVE Summary of this function goes here
%   Detailed explanation goes here

    if(nargin < 3)
        error('Not enough parameters. Usage: sendArmTrajectoryMove(netInfo, trajectory, duration)');
    end
    
    if(sum(size(trajectory) == [7 100]) ~= 2)
        error('Trajectory matrix is the wrong size');
    end
    
    % Reshape the trajectory into a column vector
    trajCol = reshape(trajectory,700,1);
    
    % Set move type constant
    TRAJ_MOVE = 2;
    data = [TRAJ_MOVE duration trajCol' trajCol(end-6:end)' trajCol(end-6:end)' zeros(1,33) ];
    sendArmMove(netInfo,data);
    
end

