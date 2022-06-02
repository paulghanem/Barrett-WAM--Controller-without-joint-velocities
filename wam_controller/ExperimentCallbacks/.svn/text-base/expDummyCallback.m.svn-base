% this should return 0 if experiment is complete
function [experiment,callbackData] = expDummyCallback(experiment, callbackData)
    experiment.state = 0;
    return;

    qhoriz = [-pi/4 -pi/2 pi/2 3*pi/4];
    qd = ones(1,4);
    qdd = ones(1,4);
    
    wout = [0.5 1 0 0];
    win = [0 1 0 0];
    
    experiment.state = 0;
    
    home_pos = callbackData.wamInfo.arm.home_norest;
    %experiment.record = 1;
    
    switch(experiment.state)
        case 1
            % joint space move to start position
            sendArmJspaceMove(callbackData.netInfo, qhoriz, qd, qdd);
            experiment.state = experiment.state+1;
        case 2
            % cartesian space move across table to 0.5 m
            if(sum(callbackData.wamDataRcvd.qdes == qhoriz) == 4)
                sendArmCspaceMove(callbackData.netInfo, '100111', wout);
                experiment.state = experiment.state+1;
            end
        case 3
            % cartesian space move back to 0 m
            if(sum(abs(callbackData.wamDataRcvd.wdes-wout) < 0.01) == 4)
                sendArmCspaceMove(callbackData.netInfo, '100111', win);
                experiment.state = experiment.state+1;
            end
        case 4
            % cartesian space move back to 0 m
            if(sum(abs(callbackData.wamDataRcvd.wdes-win) < 0.01) == 4)
                sendArmHome(callbackData.netInfo,callbackData.wamInfo);
                experiment.state = experiment.state+1;
            end
        case 5
            % stop when we get home
            if(sum(callbackData.wamDataRcvd.qdes == home_pos) == 4)
                experiment.state = 0;
            end
    end
end
