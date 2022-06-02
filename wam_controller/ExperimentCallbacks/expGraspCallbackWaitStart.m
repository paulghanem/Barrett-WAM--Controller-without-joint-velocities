function [experiment,callbackData] = expGraspCallbackWaitStart(experiment,callbackData)
    % if we didn't get track data then don't do anything
    if(isempty(callbackData.trackDataRcvd))
        return;
    end
    
    % grab the cylinder trackable
    cylinder = callbackData.trackDataRcvd.trackables(experiment.cylId);

    % compute circle location using (x,y) locations of the cylinder markers
    [p0,r] = getCircle3Points(cylinder.mapos(:,1:2));
    
    % make sure the cylinder is tracked and we're still within tolerance
    if(~cylinder.tracked || sum(abs(p0-callbackData.inputParams.cylPos) > callbackData.inputParams.cylTol) ~= 0)
        experiment.state = experiment.state-1;
        return;
    end

    % delete the previous message
    fprintf(repmat('\b',1,experiment.displen));
    
    % compute elapsed time
    elapsedTime = callbackData.recvTime-experiment.stateTime;
    
    % check if we're ready to start the experiment
    if(elapsedTime >= 5)
        % set the cylinder radius and experiment state
        % subtract tracking ball radius (11.11/2 mm) and tape thickness (2 mm)
        experiment.cylRadius = r-(0.01111/2)-0.002;
        experiment.state = experiment.state+1;
        
        % show that we're now starting
        fprintf('\b\b\b\b\bStarting the grasp with cylinder at (x=%.4f, y=%.4f), radius=%.4f\n', p0(1), p0(2),experiment.cylRadius);
        
        % compute the new trackable frame
        % determine new trackable position in the WAM base frame
        tpos_new_wam = [p0(1); p0(2); mean(cylinder.mapos(:,3))];
        
        % we know Tobjwam is defined by tqtr and tpos
        % take the inverse to find Twamobj (Shilling, pp. 47)
        Robjwam = getQuaternion2Rot(cylinder.tqtr);
        Twamobj = [Robjwam' -Robjwam'*cylinder.tpos'];
        
        % determine new trackable position in the object frame (defined by camera system)
        % these will move together since they are in the same rigid body
        tpos_new_obj = Twamobj(1:3,:)*[tpos_new_wam; 1];
        experiment.Tcylobj = [Twamobj(1:3,1:3) tpos_new_obj; 0 0 0 1];
        
        % this is Tobjwam*Tcylobj, should give [1 0 0 x; 0 1 0 y; 0 0 1 z; 0 0 0 1]
        % meaning cylinder has a zero starting orientation
%         [Robjwam cylinder.tpos'; 0 0 0 1]*experiment.Tcylobj
    else
        % show the countdown
        cylStr = sprintf('(%.4f, %.4f) -> ready (%d)', p0(1), p0(2), ceil(5-elapsedTime));
        fprintf(cylStr);
        experiment.displen = length(cylStr);
    end
end
