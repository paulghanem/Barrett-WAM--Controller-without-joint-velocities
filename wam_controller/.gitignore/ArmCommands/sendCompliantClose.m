function sendCompliantClose( netInfo, targetGripperWidth, leftOrRight, gripperSpeed )
%SENDCOMPLIANTCLOSE Sends a compliant close to the gripper and WAM
%   A compliant close is one where one finger remains fixed in world space.
%   This is achieved by moving the gripper's palm along the closing direction
%   with the same speed as the fingers are closing.

    if nargin < 4
       gripperSpeed = .05; 
    end
    
    if strcmp(leftOrRight,'left')
        leftorright = 1;
    elseif strcmp(leftOrRight,'right');
        leftorright = -1;
    else
        error('Finger to move must be either "left" or "right" finger');
    end
 
    % set COMPLIANT_CLOSE constant
    COMPLIANT_CLOSE = 3;
    
    % convert data to bytes
    data = [COMPLIANT_CLOSE targetGripperWidth gripperSpeed leftorright zeros(1,745)];
    
    % send the packet
    sendArmMove(netInfo,data);
    
    pause(.25);
       
    % send the gripper position
    sendWSGPosition(netInfo,targetGripperWidth,gripperSpeed,'absolute','clamp');
    %sendWSGGraspPart(netInfo,targetGripperWidth,gripperSpeed);

end

