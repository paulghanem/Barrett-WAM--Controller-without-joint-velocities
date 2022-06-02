function [experiment,callbackData] = expExecutePlanCallback( experiment, callbackData )
%EXPEXECUTEPLANCALLBACK Summary of this function goes here
%   Detailed explanation goes here

    qd = .5*ones(1,7);
    qdd = .5*ones(1,7);
    
    arm_ready = callbackData.wamInfo.arm.ready;
    home_pos = callbackData.wamInfo.arm.home_norest;
    
    switch(experiment.state)
        case 1 % Set up experiment, set gripper defaults
            % locate the cylinder in the list of trackables
            if(~isfield(experiment,'cylId'))
                experiment.cylId = getTrackableIdByName(experiment.trackData.trackables, 'Cylinder');

                % check if cylinder exists
                if(experiment.cylId == 0)
                    error('There is no trackable named "Cylinder"');
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
                        experiment.state = 6;
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
                       fprintf('Error in getting data1\n\n');
                       return;
                    end
                    
                    % Try to send the IK move. Error out if there was no solution
                    try
                      experiment.waypointQ = sendArmIKMove(callbackData.netInfo,Twamtip);
                      fprintf('\n1 Moving to first waypoint\n');
                      experiment.state = experiment.state + 1;
                      experiment.stateTime = callbackData.recvTime;
                      fprintf('State %d\n',experiment.state);
                    catch e
                      fprintf('No solution was found to place arm in correct position\n\n');
                      experiment.state = 6;
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
                       if ~isempty(callbackData.trackDataRcvd.trackables(experiment.cylId).Tobjwam)
                         fprintf('1.5 Plan %d - Waypoint %d\n', experiment.planNum, experiment.waypointNum);
                         % Send gripper width
                         sendWSGPosition(callbackData.netInfo,experiment.plan.plans(experiment.planNum).waypoints(experiment.waypointNum).gripWidth);  
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
               sendArmCspaceMove(callbackData.netInfo,Twamtip);
               fprintf('2 Plan %d - Waypoint %d\n', experiment.planNum, experiment.waypointNum);
               
               % Move to the next state
               experiment.state = experiment.state+1;
               experiment.stateTime = callbackData.recvTime;
            end
            
        case 5
            % When we reach a waypoint, we can close the gripper to the correct position
            
            % Here we check wddes from the recieved WAM packet.  If it is NAN, that means the controller
            % is no longer in a cartesian space move.  We wait at least 5 seconds.
            dt = callbackData.recvTime - experiment.stateTime;
            if(sum(isnan(callbackData.wamDataRcvd.wddes)) == 7 && dt > 5)
               % The controller has finished the cartesian space move to the waypoint
               fprintf('Looking For - Plan %d, Waypoint %d\n',experiment.planNum,experiment.waypointNum);
               % Send the gripper close command
               sendWSGPosition(callbackData.netInfo,experiment.plan.plans(experiment.planNum).waypoints(experiment.waypointNum).gripWidth);
               fprintf('Moving fingers to %f\n',experiment.plan.plans(experiment.planNum).waypoints(experiment.waypointNum).gripWidth);
               
               % If this was the last waypoint, prompt the user
               if(experiment.waypointNum == experiment.plan.plans(experiment.planNum).nPts) 
                  input('\nPress [Enter] to release object.\n');
                  sendWSGPosition(callbackData.netInfo,.11,.7); % Open grasp quickly
                  input('\nPress [Enter] to finish experiment');
                  experiment.state = 6;
                  experiment.stateTime = callbackData.recvTime;
               else
                  % This was not the last waypoint
                  experiment.waypointNum = experiment.waypointNum + 1;
                  
                  % If we can't find the trackable, do nothing
                  if ~isempty(callbackData.trackDataRcvd)
                     Twamtip = getTwamTip(experiment,callbackData);
                     experiment.Twamtip(:,:,experiment.waypointNum) = Twamtip;
                  else
                     return;
                  end
                  
                  %sendArmCspaceMove(callbackData.netInfo,Twamtip);
                  
                  
                  
                  fprintf('3 Plan %d - Waypoint %d\n', experiment.planNum, experiment.waypointNum);
                  
                  experiment.stateTime = callbackData.recvTime;
               end
               
            end
            
        case 6
            % Wait five seconds before doing anything
            if(callbackData.recvTime - experiment.stateTime >= 5)
                sendArmJspaceMove(callbackData.netInfo,arm_ready);
                experiment.state = experiment.state + 1;
                experiment.stateTime = callbackData.recvTime;
            end
        case 7
            % Wait to reach arm_ready then close gripper
            if(sum(callbackData.wamDataRcvd.qdes == arm_ready) == 7)
                sendWSGPosition(callbackData.netInfo,0.01,.7);
                experiment.state = experiment.state + 1;
                experiment.stateTime = callbackData.recvTime;
            end
        case 8
            % Wait to reach arm_ready, then move home
            if(sum(callbackData.wamDataRcvd.qdes == arm_ready) == 7)
                sendArmJspaceMove(callbackData.netInfo,home_pos);
                experiment.state = experiment.state + 1;
                experiment.stateTime = callbackData.recvTime;
            end
        case 9
            % stop when we get home
            if(sum(callbackData.wamDataRcvd.qdes == home_pos) == 7)
                experiment.state = 0;
            end
    end

end

function Twamtip = getTwamTip(experiment, callbackData)
    fprintf('[getTwamTip] Plan %d, Waypoint %d\n',experiment.planNum,experiment.waypointNum);
    % The transformation of the object with respect to the WAM
    Tobjwam = callbackData.trackDataRcvd.trackables(experiment.cylId).Tobjwam;
    % The desired transformation of the tool tip with respect to the object
    TobjTip = experiment.plan.plans(experiment.planNum).waypoints(experiment.waypointNum).Htransform;
    % The desired transformation of the tooltip with respect to the WAM
    Twamtip = Tobjwam * TobjTip;
end
