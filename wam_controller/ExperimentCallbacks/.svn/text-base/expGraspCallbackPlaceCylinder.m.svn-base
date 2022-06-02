function [experiment,callbackData] = expGraspCallbackPlaceCylinder(experiment,callbackData)
    % check if cylPos is set
    if(~isfield(callbackData.inputParams,'cylPos'))
        error('inputParams.cylPos is not set');
    end
    
    % check if cylTol is set
    if(~isfield(callbackData.inputParams,'cylTol'))
        error('inputParams.cylTol is not set');
    end

%     R = getQuaternion2Rot(callbackData.trackDataRcvd.trackables(experiment.cylId).tqtr)
%     experiment.state = 0;
%     return;

    % if we didn't get track data then don't do anything
    if(isempty(callbackData.trackDataRcvd))
        return;
    end
    
%     % show the plot
%     if(isfield(experiment,'fig'))
%         animateTrackingFrameRT(experiment,callbackData);
%     else
%         experiment.fig = animateTrackingFrameRT(experiment,callbackData);
%         
%         % maximize the figure window
%         %set(gcf, 'Units', 'normalized', 'Position', [0,0,1,1]);
%     end

    % backspace the old display if necessary
    if(isfield(experiment,'displen'))
        fprintf(repmat('\b',1,experiment.displen));
    else
        cylPert = callbackData.inputParams.cylPos-callbackData.inputParams.cylExpPos;
        fprintf('\nCylinder perturbation (x=%.4f, y=%.4f)\n', cylPert(1), cylPert(2));
        fprintf('Place the cylinder standing up on the table at (x=%.4f, y=%.4f)\n\n    ', callbackData.inputParams.cylPos(1), callbackData.inputParams.cylPos(2));
    end
    
    % determine the cylinder location
    accurate = 0;
    if(callbackData.trackDataRcvd.trackables(experiment.cylId).tracked)
        % compute circle location using (x,y) locations of the cylinder markers
        [p0,r] = getCircle3Points(callbackData.trackDataRcvd.trackables(experiment.cylId).mapos(:,1:2));
        cylStr = sprintf('(%.4f, %.4f)',p0(1)-callbackData.inputParams.cylPos(1),p0(2)-callbackData.inputParams.cylPos(2));
        
        % check if we're within tolerance of the desired cylinder position
        if(abs(p0-callbackData.inputParams.cylPos) < callbackData.inputParams.cylTol)
            accurate = 1;
            cylStr = [cylStr ' -> ready (5)'];
        end
    else
        cylStr = '(Not Visible)';
    end
    
    % print the cylinder location
%     cylStr = [cylStr 13 13];
    fprintf(cylStr);
    experiment.displen = length(cylStr);
    
    % move on to next state and set the start time
    if(accurate)
        experiment.state = experiment.state+1;
        experiment.stateTime = callbackData.recvTime;
    end
end
