function [experiment,callbackData] = expFTCalibrate( experiment, callbackData )
%EXPEXECUTEPLANCALLBACK Summary of this function goes here
%   Detailed explanation goes here

    qd = .5*ones(1,7);
    qdd = .5*ones(1,7);
    
    arm_ready = callbackData.wamInfo.arm.ready;
    home_pos = callbackData.wamInfo.arm.home_norest;
    
    switch(experiment.state)
        case 1 % Set up experiment
            experiment.record = 1;
            
            experiment.angles = -pi/2:pi/12:pi/2;
            experiment.index = 2;
            
            experiment.stage = 1;
            
            experiment.state = experiment.state + 1;
            experiment.stateTime = callbackData.recvTime;
            
            fprintf('State %d\n',experiment.state);
            
        case 2
            % Wait one seconds to get arm into arm_home_norest, then send to arm_ready
            if(callbackData.recvTime - experiment.stateTime > 1)
               sendArmJspaceMove(callbackData.netInfo, arm_ready, qd, qdd);
               experiment.stateTime = callbackData.recvTime;
               experiment.state = experiment.state + 1;
               fprintf('State %d\n',experiment.state);
            end
            
        case 3
            % Wait until arm gets to arm_ready, then Tare FT sensor
            if(sum(callbackData.wamDataRcvd.qdes == arm_ready) == 7)
                % Tare FT Sensor
                sendTareFTSensor(callbackData.netInfo);
                experiment.state = experiment.state + 1;
                experiment.stateTime = callbackData.recvTime;
                fprintf('State %d\n',experiment.state);
            end
        case 4
            % Wait five seconds to tare FT sensor,  then start move to initial position
            if(callbackData.recvTime - experiment.stateTime > 5)
                experiment.state = experiment.state + 1;
                experiment.qInc = [0   -0.7854         0    2.3562         -pi/2         pi/2-.1         -2.1];
                experiment.angles = linspace(-2.1,3*pi/4,20);
                experiment.index = 1;
                sendArmJspaceMove(callbackData.netInfo,experiment.qInc,qd,qdd)
                fprintf('State %d\n',experiment.state);
            end
                        
        case 5
            callbackData.wamDataRcvd.qdes
            experiment.qInc
            % Wait until in initial position then start incremental motion
            if(sum(callbackData.wamDataRcvd.qdes == experiment.qInc) == 7)
               experiment.state = experiment.state + 1;
               experiment.stateTime = callbackData.recvTime;
               experiment.qInc(7) = experiment.angles(experiment.index);
               sendArmJspaceMove(callbackData.netInfo,experiment.qInc,qd,qdd)
               fprintf('State %d\n',experiment.state);
            end

        case 6
            % Wait until we are in position then, move joint 7 incrementally
            if(sum(callbackData.wamDataRcvd.qdes == experiment.qInc) == 7)
                % If at least five seconds have passed since the last move
                if(callbackData.recvTime - experiment.stateTime >= 5)
                    experiment.stateTime = callbackData.recvTime;
                    % If we have more increments to go
                    if(experiment.index < length(experiment.angles))
                       fprintf('Moving to index %d\n',experiment.index);
                       experiment.index = experiment.index + 1;
                       experiment.qInc(7) = experiment.angles(experiment.index);
                       sendArmJspaceMove(callbackData.netInfo,experiment.qInc);
                    else
                       experiment.state = experiment.state + 1;
                       % Reset joint 7 to get ready for sweep down
                       experiment.qInc(7) = 0;
                       sendArmJspaceMove(callbackData.netInfo,experiment.qInc);
                       fprintf('State: %d\n',experiment.state);
                    end
                end
            end

        case 7
            
            % Wait until joint 7 is reset, then start incremental motion to swing gripper to the other side
            if(sum(callbackData.wamDataRcvd.qdes == experiment.qInc) == 7)
                experiment.state = experiment.state + 1;
                experiment.angles = linspace(-pi/2,1.1,15);
                experiment.index = 1;
                experiment.qInc(5) = experiment.angles(experiment.index);
                fprintf('State: %d\n',experiment.state);
            end
            
        case 8
            
            % Wait until we are in the ready position then, move joint 5 incrementally
            if(sum(callbackData.wamDataRcvd.qdes == experiment.qInc) == 7)
                % If at least five seconds have passed since the last move
                if(callbackData.recvTime - experiment.stateTime >= 5)
                    experiment.stateTime = callbackData.recvTime;
                    % If we have more increments to go
                    if(experiment.index < length(experiment.angles))
                       fprintf('Moving to index %d\n',experiment.index);
                       experiment.index = experiment.index + 1;
                       experiment.qInc(5) = experiment.angles(experiment.index);
                       sendArmJspaceMove(callbackData.netInfo,experiment.qInc);
                    else
                       experiment.state = experiment.state + 1;
                       % Rotate joint 7 to get ready for sweep down back the other way
                       experiment.qInc(7) = pi/2;
                       sendArmJspaceMove(callbackData.netInfo,experiment.qInc);
                       fprintf('State: %d\n',experiment.state);
                    end
                end
            end           
            
        case 9
            
            % Wait for Joint 7 to reach proper position, then start moving J5 back
            if(sum(callbackData.wamDataRcvd.qdes == experiment.qInc) == 7)
               experiment.state = experiment.state + 1;
               experiment.stateTime = callbackData.recvTime;
               experiment.angles =  linspace(1.1,-pi/2,12);
               experiment.index = 1;
               experiment.qInc(5) = experiment.angles(experiment.index);
               sendArmJspaceMove(callbackData.netInfo,experiment.qInc);
            end
            
        case 10
            
            % Wait until we are in the ready position then, move joint 5 incrementally
            if(sum(callbackData.wamDataRcvd.qdes == experiment.qInc) == 7)
                % If at least five seconds have passed since the last move
                if(callbackData.recvTime - experiment.stateTime >= 5)
                    experiment.stateTime = callbackData.recvTime;
                    % If we have more increments to go
                    if(experiment.index < length(experiment.angles))
                       fprintf('Moving to index %d\n',experiment.index);
                       experiment.index = experiment.index + 1;
                       experiment.qInc(5) = experiment.angles(experiment.index);
                       sendArmJspaceMove(callbackData.netInfo,experiment.qInc);
                    else
                       experiment.state = experiment.state + 1;
                       % Sendto arm_ready
                       sendArmJspaceMove(callbackData.netInfo,arm_ready);
                       fprintf('State: %d\n',experiment.state);
                    end
                end
            end     
            
        case 11
            % Wait to reach arm_ready, then move home
            if(sum(callbackData.wamDataRcvd.qdes == arm_ready) == 7)
                sendArmJspaceMove(callbackData.netInfo,home_pos);
                experiment.state = experiment.state + 1;
                experiment.stateTime = callbackData.recvTime;
                fprintf('State 10\n');
            end
        case 12
            % stop when we get home
            if(sum(callbackData.wamDataRcvd.qdes == home_pos) == 7)
                experiment.state = 0;
            end
    end

end
