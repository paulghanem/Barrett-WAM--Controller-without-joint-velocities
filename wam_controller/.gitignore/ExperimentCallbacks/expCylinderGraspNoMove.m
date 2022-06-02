function [experiment,callbackData] = expCylinderGraspNoMove( experiment, callbackData )
%EXPSAMPLE A simple callback function that extends the arm and pulls it back
%   Detailed explanation goes here

    qhoriz = [-pi/4 -pi/2 pi/2 3*pi/4-.1];
    qd = ones(1,4);
    qdd = ones(1,4);
   
    home_pos = callbackData.wamInfo.arm.home_norest;
    experiment.record = 1;
    
    % locate the cylinder in the list of trackables
    if(~isfield(experiment,'cylId'))
        experiment.cylId = getTrackableIdByName(experiment.trackData.trackables, 'Cylinder');
        
        % check if cylinder exists
        if(experiment.cylId == 0)
            error('There is no trackable named "Cylinder"');
        end
    end
    
    switch(experiment.state)
        case 1
            fprintf('State 1\n');
            % joint space move to horizontal configuration
            sendArmJspaceMove(callbackData.netInfo, qhoriz, qd, qdd);
            experiment.stateTime = callbackData.recvTime;
            experiment.state = experiment.state+1;
        case 2
            fprintf('State 2\n');
            % If there is no data found, hold still
            if(isempty(callbackData.trackDataRcvd))
                return;
            end
            % If we found the cyliner
            [p0,r] = getCircle3Points(callbackData.trackDataRcvd.trackables(experiment.cylId).mapos(:,1:2));
            experiment.objectPosition = p0;
            fprintf('Object Position %d  %d\n',p0(1),p0(2));
            
            try
                temp = invKin2DOF(p0(1),p0(2));
            catch e
                fprintf('\n%s\nPlace cylinder in workspace of arm\n\n',e.message)
                experiment.state = 10;
            end
            
            experiment.state = experiment.state + 1;
            
        case 3
            fprintf('State 3\n');
            % Give some time to get to the horizontal config, then open hand
            if(callbackData.recvTime - experiment.stateTime >= 3)
                sendHandOpen(callbackData.netInfo);
                experiment.stateTime = callbackData.recvTime;
                experiment.state = experiment.state + 1;
            end
        case 4
            fprintf('State 4\n');
            % Give two seconds to open the hand, then move in front of the cylinder position
            if(callbackData.recvTime - experiment.stateTime >= 2)
                sendArmJspaceMove(callbackData.netInfo, invKin2DOF(experiment.objectPosition(1)-.06, experiment.objectPosition(2)));
                experiment.stateTime = callbackData.recvTime;
                experiment.state = experiment.state + 1;
            end
            
        case 5
            fprintf('State 5\n');
            % Wait for user to press a key to replace cyliner before moving to final location
            % (Also be sure at least five seconds have passed)
            
            fprintf('Press a key when cylinder is in desired position');
            waitforbuttonpress
            experiment.state = experiment.state + 1;
            
        case 6
            fprintf('State 6\n');
            
            % Get new position of the cylinder
            % If there is no data found, hold still
            if(isempty(callbackData.trackDataRcvd))
                fprintf('Cannot find cylinder\n');
                return;
            end
            
            % If we found the cylinder
            [p0,r] = getCircle3Points(callbackData.trackDataRcvd.trackables(experiment.cylId).mapos(:,1:2));
            experiment.newObjectPosition = p0;
            try
                temp = invKin2DOF(p0(1),p0(2));
            catch e
                fprintf('\n%s\nPlace cylinder in workspace of arm\n\n',e.message)
                experiment.state = 9;
            end
            fprintf('New Object Position %d  %d\n',experiment.newObjectPosition(1),experiment.newObjectPosition(2));
            
            % Move palm to center of the cylinder
            %(Five second wait before moving is so that even if the user presses a key instantly, we still give
            % the arm enough time to move to position)
            if(callbackData.recvTime - experiment.stateTime >= 5)
                sendArmJspaceMove(callbackData.netInfo, invKin2DOF(experiment.newObjectPosition(1), experiment.newObjectPosition(2)));
                experiment.stateTime = callbackData.recvTime;
                experiment.state = experiment.state+1;
            end
        
        case 7
            fprintf('State 7\n');
            
            %T5to0 = getTransformN2Base(5,callbackData.wamInfo,[callbackData.wamDataRcvd.q 0]);
            %edgeOfCyl = callbackData.inputParams.cylExpPos(1)-experiment.cylRadius;
            %palmXPos = T5to0(1,4);
            %palmYPos = T5to0(2,4);
            %distToCenter = sqrt((experiment.objectPosition(1) - palmXPos)^2 + (experiment.objectPosition(2) - palmYPos)^2);
            %fprintf('Dist to center: %d\n',distToCenter);
            %if(distToCenter <= .02)
            if(callbackData.recvTime - experiment.stateTime >= 3)
                sendHandFingerClose(callbackData.netInfo);
                experiment.stateTime = callbackData.recvTime;
                experiment.state = experiment.state+1;
            end
            
        case 8
            fprintf('State 8\n');
            % Give enough time to move to the cylinder, then open hand
            if(callbackData.recvTime - experiment.stateTime >= 3)
               sendHandOpen(callbackData.netInfo);
               experiment.stateTime = callbackData.recvTime;
               experiment.state = experiment.state+1;
            end
        
        case 9
            fprintf('State 9\n');
            % Give enough time to open the hand then move back to the horizontal position
            if(callbackData.recvTime - experiment.stateTime >= 2)
               sendArmJspaceMove(callbackData.netInfo,qhoriz);
               experiment.stateTime = callbackData.recvTime;
               experiment.state = experiment.state+1;
            end
        
        case 10
            fprintf('State 10\n');
            
            % Give enough time to move to the horizontal position then close the hand
            if(callbackData.recvTime - experiment.stateTime >= 5)
                sendHandClose(callbackData.netInfo);
                experiment.stateTime = callbackData.recvTime;
                experiment.state = experiment.state+1;
            end
            
        case 11
            fprintf('State 11\n');
            % Give enough time to close the hand, then fold to home position
            if(callbackData.recvTime - experiment.stateTime >= 3)
               sendArmHome(callbackData.netInfo,callbackData.wamInfo);
               experiment.state = experiment.state+1;
            end
        case 12
            % stop when we get home
            if(sum(callbackData.wamDataRcvd.qdes == home_pos) == 4)
                experiment.state = 0;
            end
    end

end

