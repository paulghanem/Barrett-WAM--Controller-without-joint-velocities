function [experiment,callbackData] = expExecutePlanCallbackCSpace( experiment, callbackData )
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
            % Wait until arm is in ready position, and at least 2 seconds have passed 
            if(sum(callbackData.wamDataRcvd.qdes == arm_ready) == 7 && callbackData.recvTime - experiment.stateTime >= 2)
                experiment.state = experiment.state + 1;
                experiment.stateTime = callbackData.recvTime;
                sendWSGTareForceSensor(callbackData.netInfo);
                pause(.1);
                sendWSGPosition(callbackData.netInfo,.11,.7);
                pause(.1);
                sendTareFTSensor(callbackData.netInfo);
                fprintf('State %d\n',experiment.state);
            end
            
        case 3
            % Arm is in ready position, so send IK move to first waypoint.
            dt = callbackData.recvTime - experiment.stateTime;
            % Wait a second in case the object has not been found, and to tare the FT sensor
            if(callbackData.recvTime - experiment.stateTime >= 5)
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
                        Twamtip = getTwamTip(experiment, callbackData);
                        experiment.Twamtip(:,:,experiment.waypointNum) = Twamtip;
                    catch e
                       e
                       fprintf('Error in getting data1\n\n');
                       return;
                    end
                    
                    % sendArmIKMove to first waypoint
                    try
                      experiment.waypointQ = sendArmIKMove(callbackData.netInfo,Twamtip);
                      experiment.firstWaypoint = Twamtip;
                      experiment.firstWaypointQ = experiment.waypointQ;
                      fprintf('\n1 Moving to first waypoint\n');
                      experiment.state = experiment.state + 1;
                      experiment.stateTime = callbackData.recvTime;
                      fprintf('State %d\n',experiment.state);
                    catch e
                      e
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

               try               
                   %experiment.waypointQ = sendArmIKMove(callbackData.netInfo,Twamtip);
                   sendArmCspaceMove(callbackData.netInfo,Twamtip,1);
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
            % is no longer in a cartesian space move.  We wait at least 2.5 seconds.
            dt = callbackData.recvTime - experiment.stateTime;
            if(sum(isnan(callbackData.wamDataRcvd.wddes)) == 7 && dt > 2.5)
               % The controller has finished the cartesian space move to the waypoint
               fprintf('Looking For - Plan %d, Waypoint %d\n',experiment.planNum,experiment.waypointNum);
               % Send the gripper close command
               if experiment.waypointNum == experiment.plan.plans(experiment.planNum).nPts
                 fprintf('Sending Grasp Part\n');
                 %sendWSGGraspPart(callbackData.netInfo,experiment.plan.plans(experiment.planNum).waypoints(experiment.waypointNum).gripWidth);
                 sendCompliantClose(callbackData.netInfo,experiment.plan.plans(experiment.planNum).waypoints(experiment.waypointNum).gripWidth,'right');
               else
                 sendWSGPosition(callbackData.netInfo,experiment.plan.plans(experiment.planNum).waypoints(experiment.waypointNum).gripWidth,.05,'absolute','clamp');
               end
               
               % If this was the last waypoint, prompt the user
               if(experiment.waypointNum == experiment.plan.plans(experiment.planNum).nPts)
                  %input('\nPress [Enter] to pick up object\n');
                  %experiment.graspedQ = callbackData.wamDataRcvd.q;
                  %experiment.pickupQ = experiment.graspedQ;
                  
                  % Move joint 2 towards zero by .3 radians
                  %experiment.pickupQ(2) = experiment.pickupQ(2) - sign(experiment.pickupQ(2))*.3;
                  
                  %sendArmJspaceMove(callbackData.netInfo,experiment.pickupQ);
                  
                  input('\nPress [Enter] to put down object\n');
                  
                  %sendArmJspaceMove(callbackData.netInfo,experiment.graspedQ);
                  
                  try               
                    %experiment.waypointQ = sendArmIKMove(callbackData.netInfo,Twamtip);
                    sendArmCspaceMove(callbackData.netInfo,experiment.Twamtip(:,:,end-1),1);
                  catch e
                   fprintf('Error in putting object down\n\n');
                   experiment.state = 6;
                   fprintf('State %d\n',experiment.state);
                   return;
                  end
                  
                  input('\nPress [Enter] to release object.\n');
                  sendWSGReleasePart(callbackData.netInfo,.08,.7); % Release the object quickly
                  %sendWSGPosition(callbackData.netInfo,.11,.7); % Open grasp quickly
                  
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
                  
                  try               
                    %experiment.waypointQ = sendArmIKMove(callbackData.netInfo,Twamtip);
                    sendArmCspaceMove(callbackData.netInfo,Twamtip,1);
                  catch e
                   fprintf('2 No solution was found to place arm in correct position\n\n');
                   experiment.state = 6;
                   fprintf('State %d\n',experiment.state);
                   return;
                  end

                  fprintf('3 Plan %d - Waypoint %d\n', experiment.planNum, experiment.waypointNum);
                  
                  experiment.stateTime = callbackData.recvTime;
                  fprintf('State %d\n',experiment.state);
               end
            end
            
        case 6
            % Wait five seconds before doing anything
            if(callbackData.recvTime - experiment.stateTime >= 2.5)
                %sendArmJspaceMove(callbackData.netInfo,experiment.firstWaypointQ);
                sendArmCspaceMove(callbackData.netInfo,experiment.firstWaypoint,0);
                experiment.state = experiment.state + 1;
                experiment.stateTime = callbackData.recvTime;
                fprintf('State 7\n');
            end
            
        case 7
            dt = callbackData.recvTime - experiment.stateTime;
            % Wait to finish C space move, then more to arm_ready
            if(sum(isnan(callbackData.wamDataRcvd.wddes)) == 7 && dt > 2.5)
                input('Press [Enter] to send arm to ready position and then home\n');
                sendArmJspaceMove(callbackData.netInfo,arm_ready);
                experiment.state = experiment.state + 1;
                experiment.stateTime = callbackData.recvTime;
                fprintf('State 8\n');
            end
        case 8
            % Wait to reach arm_ready then close gripper
            if(sum(callbackData.wamDataRcvd.qdes == arm_ready) == 7)
                sendWSGPosition(callbackData.netInfo,0.001,.7);
                experiment.state = experiment.state + 1;
                experiment.stateTime = callbackData.recvTime;
                fprintf('State 9\n');
            end
        case 9
            % Wait to reach arm_ready, then move home
            if(sum(callbackData.wamDataRcvd.qdes == arm_ready) == 7)
                sendArmJspaceMove(callbackData.netInfo,home_pos);
                experiment.state = experiment.state + 1;
                experiment.stateTime = callbackData.recvTime;
                fprintf('State 10\n');
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
    TWAMTray = callbackData.trackDataRcvd.trackables(experiment.objId).Tobjwam
    experiment.TWAMTray = TWAMTray;
    
    TTrayObj = getTTrayObj(experiment.plan.objectId)
    
    % The desired transformation of the tool tip with respect to the object
    TobjTip = experiment.plan.plans(experiment.planNum).waypoints(experiment.waypointNum).Htransform
    experiment.TobjTip = TobjTip;
    % The desired transformation of the tooltip with respect to the WAM
    Twamtip = TWAMTray * TTrayObj * TobjTip
    experiment.Twamtip = Twamtip;
end
