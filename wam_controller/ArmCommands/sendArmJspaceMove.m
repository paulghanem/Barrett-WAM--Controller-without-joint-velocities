% recommended qd=0.5, qdd=0.5
function sendArmJspaceMove(netInfo,q,qd,qdd)
% SENDARMJSPACEMOVE Sends a joint space move to the arm.
% Sends a joint space move to the arm.  q is the desired joint angles. qd is the
% maximum joint velocities, qdd is the maximum joint accelerations

    % check that q has appropriate length and type
    if(~strcmp(class(q(1)), 'double') || sum(size(q) == [1 7]) ~= 2)
        error('invalid data format for q');
    end
    
    % set default velocity/accel if either is not given
    if(nargin < 4)
        qd = 0.5*ones(1,7);
        qdd = 0.5*ones(1,7);
    else
        % check that qd has appropriate length and type
        if(~strcmp(class(qd(1)), 'double') || length(qd) ~= 7)
            error('invalid data format for qd');
        end

        % check that qdd has appropriate length and type
        if(~strcmp(class(qdd(1)), 'double') || length(qdd) ~= 7)
            error('invalid data format for qdd');
        end
    end

    % set JOINT_MOVE constant
    JOINT_MOVE = 0;
    
    % convert data to bytes
    data = [JOINT_MOVE q qdd qd zeros(1,727)];
    
    % send the packet
    sendArmMove(netInfo,data);
end
