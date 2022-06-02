% this should return 0 if experiment is complete
function [experiment,callbackData] = expGraspCallback(experiment, callbackData)
    % qarc = the offset of the arc due to the elbow geometry
    % qexp = the distance to expand the arc in both directions
    qarc = [.0816 0 0 .0816];
    qexp = [.1 0 0 .1];

    qstart = [-pi/4 -pi/2 pi/2 3*pi/4]+qarc+qexp;
    qend = [-3*pi/4 -pi/2 pi/2 pi/4]+qarc-qexp;
    qd = ones(1,4);
    qdd = ones(1,4);
    
    % accelerate fast into the grasp
    qddg = 3*ones(1,4);
    
    % locate the cylinder in the list of trackables
    if(~isfield(experiment,'cylId'))
        experiment.cylId = getTrackableIdByName(experiment.trackData.trackables, 'Bottle');
        
        % check if cylinder exists
        if(experiment.cylId == 0)
            error('There is no trackable named "Bottle"');
        end
    end
    
    % check if palmDist is set
    if(~isfield(callbackData.inputParams,'palmDist'))
        error('inputParams.palmDist is not set');
    end
    
    % check if cylExpPos is set
    if(~isfield(callbackData.inputParams,'cylExpPos'))
        error('inputParams.cylExpPos is not set');
    end
    
    switch(experiment.state)
        case 1
            [experiment,callbackData] = expGraspCallbackPlaceCylinder(experiment,callbackData);
        case 2
            [experiment,callbackData] = expGraspCallbackWaitStart(experiment,callbackData);
%             % just use default max VA (100,4)
%         case 3
%             % set max vel=0.5 rad/s and max acc=2 rad/s^2
%             sendHandFingerMaxVA(callbackData.netInfo,3581,14324);
%             experiment.state = experiment.state+1;
        case 3
            % joint space move to start position
            sendArmJspaceMove(callbackData.netInfo, [0 0 0 0], qd, qdd);
            experiment.stateTime = callbackData.recvTime;
            experiment.state = experiment.state+1;
        case 4
            % wait 1 sec, then send the hand open
            if(callbackData.recvTime - experiment.stateTime >= 1)
                sendHandOpen(callbackData.netInfo);
                experiment.stateTime = callbackData.recvTime;
                experiment.state = experiment.state+1;
            end
        case 5
            % wait 2 sec for hand to open, then send arm to end
            if(callbackData.recvTime - experiment.stateTime >= 2)
                experiment.record = 1;
                sendArmJspaceMove(callbackData.netInfo, [0 pi/2 pi/2 pi/2], qd, qddg);
                experiment.stateTime = callbackData.recvTime;
                experiment.state = experiment.state+1;
            end
%             edgeOfCyl = callbackData.inputParams.cylPos(1)-experiment.cylRadius;
%             sendArmCspaceMove(callbackData.netInfo, 1, [edgeOfCyl -1.2092 1.2092 -1.2092]);
%             experiment.state = 0;
        case 6
            if(sum(callbackData.wamDataRcvd.qdes == [0 pi/2 pi/2 pi/2]) == 4)
                experiment.record = 0;
                sendHandFingerClose(callbackData.netInfo);
                experiment.stateTime = callbackData.recvTime;
                experiment.state = experiment.state+1;
            end
            
            % wait until palm is palmDist away, then send the hand close
            %if(callbackData.recvTime - experiment.stateTime >= 0.6)
            %T5to0 = getTransformN2Base(5,callbackData.wamInfo,[callbackData.wamDataRcvd.q 0]);
            %edgeOfCyl = callbackData.inputParams.cylExpPos(1)-experiment.cylRadius;
            %palmXPos = T5to0(1,4);
            
            %if(edgeOfCyl-palmXPos <= callbackData.inputParams.palmDist)
            %    sendHandFingerClose(callbackData.netInfo);
            %    experiment.actPalmDist = edgeOfCyl-palmXPos;
            %    experiment.state = experiment.state+1;
            %    fprintf('Starting finger close with actual gap=%.4f (desired gap=%.4f)\n', experiment.actPalmDist, callbackData.inputParams.palmDist);
            %end
        case 7
            if(callbackData.recvTime - experiment.stateTime >= 2)
                experiment.record = 1;
                sendArmJspaceMove(callbackData.netInfo, [.74237 .69866 .03622 1.64187], qd, qddg);
                experiment.stateTime = callbackData.recvTime;
                experiment.state = experiment.state+1;
            end
            
            % wait for arm to get to qend
            %if(sum(callbackData.wamDataRcvd.qdes == qend) == 4)
            %    experiment.record = 0;
            %    sendArmJspaceMove(callbackData.netInfo, [-2.1885 -pi/2 pi/2 1.8530], qd, qdd);
                
            %    experiment.stateTime = callbackData.recvTime;
            %    experiment.state = experiment.state+1;
            %end
        case 8
            % Wait for arm to get to square
            % if(sum(callbackData.wamDataRcvd.qdes == [-2.1885 -1.6381 1.5933 1.8530]) == 4 && ...
            if(sum(callbackData.wamDataRcvd.qdes == [.74237 .69866 .03622 1.64187]) == 4 && ...
                    callbackData.recvTime - experiment.stateTime >= 3)
                sendHandOpen(callbackData.netInfo);
                experiment.state = experiment.state+1;
            end
        case 9
            % wait 0.5 sec, then send the arm to qstart
            if(callbackData.recvTime - experiment.stateTime >= 5)
                sendArmJspaceMove(callbackData.netInfo, [0 pi/2 pi/2 pi/2], qd, qdd);
                experiment.stateTime = callbackData.recvTime;
                experiment.state = experiment.state+1;
            end
        case 10
            % wait 1 sec, then close fingers
            if(callbackData.recvTime - experiment.stateTime >= 3)
                sendHandOpen(callbackData.netInfo);
                experiment.stateTime = callbackData.recvTime;
                
                experiment.state = experiment.state+1;
            end
        
        case 11
            % wait 0.01 sec, then open hand
            if(callbackData.recvTime - experiment.stateTime >= 1)
                sendArmJspaceMove(callbackData.netInfo, [0 0 0 0], qd, qdd);
                experiment.stateTime = callbackData.recvTime;
                experiment.state = experiment.state+1;
            end
        case 12
            % wait 0.01 sec, then close hand
            if(sum(callbackData.wamDataRcvd.qdes == [0 0 0 0]) == 4 && ...
                    callbackData.recvTime - experiment.stateTime >= 3)
                sendArmHome(callbackData.netInfo, callbackData.wamInfo);
                experiment.state = experiment.state+1;
            end
        case 13
            % stop when we get home
            if(sum(callbackData.wamDataRcvd.qdes == callbackData.wamInfo.arm.home_norest) == 4)
                experiment.state = 0;
                
                % make sure cylinder was tracked throughout the grasp
                if(~isempty(find(experiment.trackData.trackables(experiment.cylId).lastTrack(1:experiment.frameCount)~=0,1)))
                    error('Cylinder was not tracked throughout the grasp');
                end
                
                % remove temporary fields
                experiment = rmfield(experiment, {'stateTime' 'cylId' 'displen' 'Tcylobj' 'cylRadius'});
                
                fprintf('\n');
            end
            
            % need to move mbase points into the cylinder frame
            % as well as change tpos, tqtr on every iteration
    end
end
