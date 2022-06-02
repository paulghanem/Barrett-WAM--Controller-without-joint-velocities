function qNew = sendArmIKMove( netInfo,T ,force)
%SENDARMIKMOVE Sends a joint-space motion to the desired tool configuration
%   T must be a 4x4 homogenous transformation matrix.  This function will compute the inverse
%   kinematics and prompt the user if the resulting angles are okay to send to the arm.
%   The third parameter is optional and if set to 'force' will send the result of the inverse
%   kinematics without prompting the user first.

    % Get the information back from the arm so we can get current joint angles
    % to initialize inverse kinematics search. If no packet was recieved, use
    % zero configuration
    try
       [wamDataRcvd, recvTime, blockNumRcvd] = getWamPacket(netInfo);
       q = wamDataRcvd.q;
    catch e
       fprintf('\n');
       disp('Could not recieve current joint angles from WAM');
       disp('Using Zero Configuration as starting point for inverse kinematics');
       fprintf('\n');
       q = [0 0 0 0 0 0 0];
    end
    
    % Compute the inverse kinematics
    %[qNew, validResult] = ikine7DOF(q,T);
    [validResult qNew] = wam7ik_w_joint_limits(T,q');
    qNew = qNew';
    % Check for a solution and prompt user to verify angles if 'force' was not set
    if validResult
        if nargin < 3
            disp('Sending these joint angles:');
            disp(qNew);
    
            r = input('Are these joint angles okay? (y/n) ','s');
            if r == 'y' || r == 'Y'
                sendArmJspaceMove(netInfo,qNew);
            else
                fprintf('\nJoint angles were not sent\n\n');
            end
        elseif strcmp(force,'force')
            sendArmJspaceMove(netInfo,qNew);
        else
            error('Command was not sent. Check input arguments.');
        end
    else
        error('No solution was found for the input transform. No command sent.')
    end

end
