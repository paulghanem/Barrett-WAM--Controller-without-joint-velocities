function [experiment,callbackData] = expSample( experiment, callbackData )
%EXPSAMPLE A simple callback function that extends the arm and pulls it back
%   Detailed explanation goes here

    qhoriz = [-pi/4 -pi/2 pi/2 3*pi/4];
    qd = ones(1,4);
    qdd = ones(1,4);
    
    % First element of wout and win is distance along the specified axis,
    % The last three elements are the axis-angle representation of the 
    % tool configuration
    wout = [0.5 -1.2092 1.2092 -1.2092];
    win = [-0.1 -1.2092 1.2092 -1.2092];
    
    home_pos = callbackData.wamInfo.arm.home_norest;
    experiment.record = 1;
    
    % locate the cylinder in the list of trackables
    if(~isfield(experiment,'cylId'))
        experiment.cylId = getTrackableIdByName(experiment.trackData.trackables, 'Bottle');
        
        % check if cylinder exists
        if(experiment.cylId == 0)
            error('There is no trackable named "Bottle"');
        end        
    end
    
    switch(experiment.state)
        case 1
            fprintf('State 1\n');
            % joint space move to zero configuration
            sendArmJspaceMove(callbackData.netInfo, [0 0 0 0], qd, qdd);
            %sendArmJspaceMove(callbackData.netInfo, home_pos);
            experiment.stateTime = callbackData.recvTime;
            experiment.state = experiment.state+1;
        case 2
            fprintf('State 2\n');
            % If there is no data found, hold still
            if(isempty(callbackData.trackDataRcvd))
                return;
            end
            % If we found the bottle
            [p0,r] = getCircle3Points(callbackData.trackDataRcvd.trackables(experiment.cylId).mapos(:,1:2));
            experiment.objectPosition = p0;
            fprintf('Object Position %d  %d\n',p0(1),p0(2));
            
            experiment.state = experiment.state + 1;
            
        case 3
            fprintf('State 3\n');
            % Give some time to get to the zero config, then point to bottle
            % An XYZ move to 30cm above the bottle
            if(callbackData.recvTime - experiment.stateTime >= 5)
                sendArmJspaceMove(callbackData.netInfo, XYZ4DOFInvKin(experiment.objectPosition(1), experiment.objectPosition(2),.3));
                experiment.stateTime = callbackData.recvTime;
                experiment.state = 5;
            end
        case 4
            fprintf('State 4\n');
            % Wait a tiny bit of time for a button press, if we have one then start the end sequence
            
            % NEED TO FIGURE OUT HOW TO CHECK FOR A KEY PRESS WITHOUT BLOCKING
            
            % No key press
            
            % If there is no data found, hold still
            if(isempty(callbackData.trackDataRcvd))
                return;
            end
            % If we found the bottle, update the position and move to it
            [p0,r] = getCircle3Points(callbackData.trackDataRcvd.trackables(experiment.cylId).mapos(:,1:2));
            experiment.objectPosition = p0;
            fprintf('Object Position %d  %d\n',p0(1),p0(2));
            sendArmJspaceMove(callbackData.netInfo, XYZ4DOFInvKin(experiment.objectPosition(1), experiment.objectPosition(2),.3));
            experiment.state = 4;
            
        case 5
            fprintf('State 5\n');
            % cartesian space move back to 0 m
            %abs(callbackData.wamDataRcvd.wdes-wout)
            
            %if(sum(abs(callbackData.wamDataRcvd.wdes-wout) < 0.01) == 4)
            % move arm to zero config
            if(callbackData.recvTime - experiment.stateTime >= 5)
                sendArmJspaceMove(callbackData.netInfo, [0 0 0 0]);
                experiment.stateTime = callbackData.recvTime;
                experiment.state = experiment.state+1;
            end
            
        case 6
            fprintf('State 6\n');
            % cartesian space move back to 0 m
            %if(sum(abs(callbackData.wamDataRcvd.wdes-win) < 0.01) == 4)
            % wait 5 sec to move to zero config then move home
            if(callbackData.recvTime - experiment.stateTime >= 5)
                sendArmHome(callbackData.netInfo,callbackData.wamInfo);
                experiment.stateTime = callbackData.recvTime;
                experiment.state = experiment.state+1;
            end
        case 7
            fprintf('State 7\n');
            % stop when we get home
            if(sum(callbackData.wamDataRcvd.qdes == home_pos) == 4)
                experiment.state = 0;
            end
    end

end

