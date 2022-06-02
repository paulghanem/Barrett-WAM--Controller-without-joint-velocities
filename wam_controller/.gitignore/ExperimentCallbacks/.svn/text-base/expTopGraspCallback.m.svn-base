% this should return 0 if experiment is complete
function [experiment,callbackData] = expTopGraspCallback(experiment, callbackData)
    %experiment.state = 1;

    qd = ones(1,4) * .1;
    qdd = ones(1,4) * .1;
    
    home_pos = callbackData.wamInfo.arm.home_norest;
    experiment.record = 1;
    
    switch(experiment.state)
        case 1
            % joint space move to start position
            sendArmJspaceMove(callbackData.netInfo, [0 0 0 0], qd, qdd);
            experiment.stateTime = callbackData.recvTime;
            experiment.state = experiment.state+1;
            
        case 2
            disp('state 2');
            % cartesian space move across table to 0.5 m
            if(callbackData.recvTime - experiment.stateTime >= 40)
                sendArmJspaceMove(callbackData.netInfo, [0 0 0 2.3], qd, qdd);
                experiment.state = experiment.state+1;
                experiment.stateTime = callbackData.recvTime;
            end
            
        case 3
            if(callbackData.recvTime - experiment.stateTime >= 20)
                sendHandOpen(callbackData.netInfo); 
                experiment.state = experiment.state+1;
                experiment.stateTime = callbackData.recvTime;
            end
            
        case 4
            disp('state 4')
            % cartesian space move back to 0 m
            if(callbackData.recvTime - experiment.stateTime >= 2)
                sendArmJspaceMove(callbackData.netInfo, [0 (pi/4+.04) 0 2.3], qd, qdd);
                experiment.state = experiment.state+1;
                experiment.stateTime = callbackData.recvTime;
            end
            
        case 5
            disp('state 5')
            % cartesian space move back to 0 m
            if(callbackData.recvTime - experiment.stateTime >= 20)
                sendHandFingerMove(callbackData.netInfo, pi/2 - .4, '111')
                experiment.state = experiment.state+1;
                experiment.stateTime = callbackData.recvTime;
            end
        case 6
            if(callbackData.recvTime - experiment.stateTime >= 4)
                sendArmJspaceMove(callbackData.netInfo, [0 0 0 2.3], qd, qdd);
                experiment.state = experiment.state+1;
                experiment.stateTime = callbackData.recvTime;
            end
        case 7
            if(callbackData.recvTime - experiment.stateTime >= 20)
                sendArmJspaceMove(callbackData.netInfo, [0 (pi/4 + .04) 0 2.3], qd, qdd);
                experiment.state = experiment.state+1;
                experiment.stateTime = callbackData.recvTime;
            end
        case 8
            if(callbackData.recvTime - experiment.stateTime >= 20)
                
                sendHandOpen(callbackData.netInfo); 
                experiment.state = experiment.state+1;
                experiment.stateTime = callbackData.recvTime;
            end
        case 9 
            if(callbackData.recvTime - experiment.stateTime >= 4)
                sendArmJspaceMove(callbackData.netInfo, [0 0 0 2.3], qd, qdd);
                experiment.state = experiment.state+1;
                experiment.stateTime = callbackData.recvTime;
            end
        case 10
            if(callbackData.recvTime - experiment.stateTime >= 20)
                %sendHandFingerMove(callbackData.netInfo, pi/2-.6, '111')
                sendHandClose(callbackData.netInfo);
                experiment.state = experiment.state+1;
                experiment.stateTime = callbackData.recvTime;
            end
        case 11
            if(callbackData.recvTime - experiment.stateTime >= 2)
                sendArmJspaceMove(callbackData.netInfo, [0 0 0 0], qd, qdd);
                experiment.state = experiment.state+1;
                experiment.stateTime = callbackData.recvTime;
            end
        case 12
            % stop when we get home
            if(callbackData.recvTime - experiment.stateTime >= 20)
                sendArmJspaceMove(callbackData.netInfo, home_pos, qd, qdd);
                if(sum(callbackData.wamDataRcvd.qdes == home_pos) == 4)
                    experiment.state = 0;
                end
            end
    end
end