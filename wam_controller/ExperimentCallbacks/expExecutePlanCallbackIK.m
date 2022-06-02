function [experiment,callbackData] = expExecutePlanCallbackIK( experiment, callbackData )
%EXPEXECUTEPLANCALLBACK Summary of this function goes here
%   Detailed explanation goes here

    qd = .5*ones(1,7);
    qdd = .5*ones(1,7);
    
    arm_ready = callbackData.wamInfo.arm.ready;
    home_pos = callbackData.wamInfo.arm.home_norest;
    
    switch(experiment.state)
        case 1 % Set up experiment, set gripper defaults
            % locate the cylinder in the list of trackables
            if(~isfield(experiment,'objId'))
                experiment.objId = getTrackableIdByName(experiment.trackData.trackables, 'Tray');

                % check if cylinder exists
                if(experiment.objId == 0)
                    error('There is no trackable named "Tray"');
                end
            end
            experiment.plan = loadGraspPlan(callbackData.inputParams.planName);            
            experiment.planNum = callbackData.inputParams.planToRun;
            experiment.waypointNum = 1;

            sendWSGHoming(callbackData.netInfo,'negative'); % Homing to closed position
            
            experiment.stateTime = callbackData.recvTime;
            experiment.state = experiment.state + 1;
            experiment.Twamtip = zeros(4,4,experiment.plan.plans(experiment.planNum).nPts);
            
            if(experiment.planNum > experiment.plan.numberOfPlans || experiment.planNum < 1)
                fprintf('\nThe specified planToRun was not valid.\nExperiment is ending.\n');
                experiment.state = 0;
                return;
            else
                fprintf('\n Plan Number: %d\n Number of waypoints: %d\n',experiment.planNum, experiment.plan.plans(experiment.planNum).nPts);
            end
            
            experiment.record = 1;
            
            % joint space move to ready configuration
            sendArmJspaceMove(callbackData.netInfo, arm_ready, qd, qdd);
            
            fprintf('State %d\n',experiment.state);
    
        case 2
            % Wait until arm is in ready position
            if(sum(callbackData.wamDataRcvd.qdes == arm_ready) == 7)
                experiment.state = experiment.state + 1;
                experiment.stateTime = callbackData.recvTime;
                sendWSGPosition(callbackData.netInfo,.11,.7);
                fprintf('State %d\n',experiment.state);
            end
            
        case 3
            % Arm is in ready position, so send IK move to first waypoint.            
            dt = callbackData.recvTime - experiment.stateTime;
            % Wait a second in case the object has not been found
            if(callbackData.recvTime - experiment.stateTime >= 1)
                % If there is no tracking data found, hold still, do nothing
                if(isempty(callbackData.trackDataRcvd))
                    fprintf('No Data found. (%d)\n',floor(60 - dt));
                    % If we have not found the cylinder in 60 seconds, abort all plans
                    if(dt > 60)
                        fprintf('\nExperiment timed out.\n');
                        experiment.state = 7;
                        experiment.stateTime = callbackData.recvTime;
                        fprintf('State %d\n',experiment.state);
                        return;
                    end
                    return;
                % We have tracking data
                else
                    experiment.stateTime = callbackData.recvTime;
                    fprintf('1 Plan %d - Waypoint %d\n', experiment.planNum, experiment.waypointNum);
                    % If we have tracking data, use the object orientation and position to get the
                    % desired end effector orientation and position
                    try
                        Twamtip = getTwamTip(experiment, callbackData)
                        experiment.Twamtip(:,:,experiment.waypointNum) = Twamtip;
                    catch e
                       fprintf('Error in getting data1\n\n');
                       return;
                    end
                    
                    % Try to send the IK move. Error out if there was no solution
                    try
                      fprintf('Trying sendArmIKMove\n');
                      Twamtip
                      experiment.waypointQ = sendArmIKMove(callbackData.netInfo,Twamtip);
                      fprintf('sendArmIKMove Returned\n');
                      experiment.firstWaypointQ = experiment.waypointQ;
                      fprintf('\n1 Moving to first waypoint\n');
                      experiment.state = experiment.state + 1;
                      experiment.stateTime = callbackData.recvTime;
                      fprintf('State %d\n',experiment.state);
                    catch e
                      fprintf('0 No solution was found to place arm in correct position\n\n');
                      experiment.state = 7;
                      fprintf('State %d\n',experiment.state);
                      return;
                    end
                end
            end
            
        case 4
            % When we reach the first waypoint, close the gripper to the correct width
            % Send a Cartesian space move to the second waypoint.
            if(sum(callbackData.wamDataRcvd.qdes == experiment.waypointQ) == 7)
               % If we can't find the trackable, do nothing
               if ~isempty(callbackData.trackDataRcvd)
                   if ~isempty(callbackData.trackDataRcvd.trackables)
                       if ~isempty(callbackData.trackDataRcvd.trackables(experiment.objId).Tobjwam)
                         fprintf('1.5 Plan %d - Waypoint %d\n', experiment.planNum, experiment.waypointNum);
                         % Send gripper width
                         sendWSGPosition(callbackData.netInfo,experiment.plan.plans(experiment.planNum).waypoints(experiment.waypointNum).gripWidth,.05,'absolute','clamp');  
                         % Increment the waypoint number
                         experiment.waypointNum = experiment.waypointNum + 1;
                         
                         Twamtip = getTwamTip(experiment,callbackData);
                         experiment.Twamtip(:,:,experiment.waypointNum) = Twamtip;
                       end
                   end
               else
                  return;
               end
               
               % Send the cartesian space move to Twamtip
               %sendArmCspaceMove(callbackData.netInfo,Twamtip);
               try               
                   experiment.waypointQ = sendArmIKMove(callbackData.netInfo,Twamtip);
               catch e
                   fprintf('1 No solution was found to place arm in correct position\n\n');
                   experiment.state = 6;
                   fprintf('State %d\n',experiment.state);
                   return;
               end
               
               fprintf('2 Plan %d - Waypoint %d\n', experiment.planNum, experiment.waypointNum);
               
               % Move to the next state
               experiment.state = experiment.state+1;
               experiment.stateTime = callbackData.recvTime;
            end
            
        case 5
            % When we reach a waypoint, we can close the gripper to the correct position
            
            % Here we check wddes from the recieved WAM packet.  If it is NAN, that means the controller
            % is no longer in a cartesian space move.  We wait at least 5 seconds.
            if(sum(callbackData.wamDataRcvd.qdes == experiment.waypointQ) == 7)
               % The controller has finished the cartesian space move to the waypoint
               fprintf('Looking For - Plan %d, Waypoint %d\n',experiment.planNum,experiment.waypointNum);
               % Send the gripper close command
               if experiment.waypointNum == experiment.plan.plans(experiment.planNum).nPts
                 fprintf('Sending Grasp Part\n');
                 sendWSGGraspPart(callbackData.netInfo,experiment.plan.plans(experiment.planNum).waypoints(experiment.waypointNum).gripWidth);
               else
                 sendWSGPosition(callbackData.netInfo,experiment.plan.plans(experiment.planNum).waypoints(experiment.waypointNum).gripWidth,.05,'absolute','clamp');
               end
               fprintf('Moving fingers to %f\n',experiment.plan.plans(experiment.planNum).waypoints(experiment.waypointNum).gripWidth);
               
               % If this was the last waypoint, prompt the user
               if(experiment.waypointNum == experiment.plan.plans(experiment.planNum).nPts)
                  input('\nPress [Enter] to pick up object\n');
                  experiment.pickupQ = experiment.waypointQ;
                  experiment.pickupQ(2) = experiment.pickupQ(2) - .3;
                  
                  sendArmJspaceMove(callbackData.netInfo,experiment.pickupQ);
                  
                  input('\nPress [Enter] to put down object\n');
                  
                  sendArmJspaceMove(callbackData.netInfo,experiment.waypointQ);
                  
                  input('\nPress [Enter] to release object.\n');
                  sendWSGReleasePart(callbackData.netInfo,.11,.7); % Release the object quickly
                  %sendWSGPosition(callbackData.netInfo,.11,.7); % Open grasp quickly
                  input('\nPress [Enter] to finish experiment');
                  experiment.state = 6;
                  experiment.stateTime = callbackData.recvTime;
               else
                  % This was not the last waypoint
                  
                  % If we can't find the trackable, do nothing
                  if ~isempty(callbackData.trackDataRcvd)
                     experiment.waypointNum = experiment.waypointNum + 1;
                     Twamtip = getTwamTip(experiment,callbackData);
                     experiment.Twamtip(:,:,experiment.waypointNum) = Twamtip;
                  else
                     return;
                  end
                  
                  
                  %sendArmCspaceMove(callbackData.netInfo,Twamtip);
                  try               
                    experiment.waypointQ = sendArmIKMove(callbackData.netInfo,Twamtip);
                  catch e
                   fprintf('2 No solution was found to place arm in correct position\n\n');
                   experiment.state = 6;
                   fprintf('State %d\n',experiment.state);
                   return;
                  end

                  fprintf('3 Plan %d - Waypoint %d\n', experiment.planNum, experiment.waypointNum);
                  
                  experiment.stateTime = callbackData.recvTime;
               end
               
            end
            
        case 6
            % Wait five seconds before doing anything
            if(callbackData.recvTime - experiment.stateTime >= 5)
                sendArmJspaceMove(callbackData.netInfo,experiment.firstWaypointQ);
                experiment.state = experiment.state + 1;
                experiment.stateTime = callbackData.recvTime;
            end
            
        case 7
            % Wait five seconds before doing anything
            if(callbackData.recvTime - experiment.stateTime >= 5)
                sendArmJspaceMove(callbackData.netInfo,arm_ready);
                experiment.state = experiment.state + 1;
                experiment.stateTime = callbackData.recvTime;
            end
        case 8
            % Wait to reach arm_ready then close gripper
            if(sum(callbackData.wamDataRcvd.qdes == arm_ready) == 7)
                sendWSGPosition(callbackData.netInfo,0.0,.7);
                experiment.state = experiment.state + 1;
                experiment.stateTime = callbackData.recvTime;
            end
        case 9
            % Wait to reach arm_ready, then move home
            if(sum(callbackData.wamDataRcvd.qdes == arm_ready) == 7)
                sendArmJspaceMove(callbackData.netInfo,home_pos);
                experiment.state = experiment.state + 1;
                experiment.stateTime = callbackData.recvTime;
            end
        case 10
            % stop when we get home
            if(sum(callbackData.wamDataRcvd.qdes == home_pos) == 7)
                experiment.state = 0;
            end
    end

end

function Twamtip = getTwamTip(experiment, callbackData)
    fprintf('[getTwamTip] Plan %d, Waypoint %d\n',experiment.planNum,experiment.waypointNum);
    % The transformation of the tray with respect to the WAM
    TWAMTray = callbackData.trackDataRcvd.trackables(experiment.objId).Tobjwam;
    experiment.TWAMTray = TWAMTray;
    
    TTrayCylinder = [1  0  0  0.03175;
                     0  1  0    0.127;
                     0  0  1  0.00635+0.0127+.008;
                     0  0  0        1];
                 
    TTrayCube = [1  0  0  0.0381;
                 0  1  0  0.0127;
                 %0  1  0  .03;
                 0  0  1  0.0508;
                 0  0  0       1];
             
    TTrayUChannel = [ 0  1  0  0.0053975;
                     -1  0  0    0.00635;
                      0  0  1     0.0508+.020;
                      0  0  0          1];
    

    % Select the object that we are trying to pick up
    TTrayObj = TTrayCube;
    
    % The desired transformation of the tool tip with respect to the object
    TobjTip = experiment.plan.plans(experiment.planNum).waypoints(experiment.waypointNum).Htransform;
    experiment.TobjTip = TobjTip;
    % The desired transformation of the tooltip with respect to the WAM
    Twamtip = TWAMTray * TTrayObj * TobjTip;
    experiment.Twamtip = Twamtip;
end
