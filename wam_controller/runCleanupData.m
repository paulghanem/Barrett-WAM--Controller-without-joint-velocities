function [experiment,avgFrameRate,avgFrameRateCam] = runCleanupData(experiment)
    % truncate frameTimes
    experiment.frameTimes = experiment.frameTimes(1:experiment.frameCount);
    
    % truncate rawMarkers to the number of max raw markers in any given
    % frame, and to frameCount
    maxRawMarkerCount = max(sum(~isnan(experiment.trackData.rawMarkers(:,1,:))));
    experiment.trackData.rawMarkers = experiment.trackData.rawMarkers(1:maxRawMarkerCount,:,1:experiment.frameCount);
    
    % truncate wamData to the frameCount
    experiment.wamData.q = experiment.wamData.q(1:experiment.frameCount,:);
    experiment.wamData.qd = experiment.wamData.qd(1:experiment.frameCount,:);
    experiment.wamData.qdes = experiment.wamData.qdes(1:experiment.frameCount,:);
    experiment.wamData.qddes = experiment.wamData.qddes(1:experiment.frameCount,:);
    experiment.wamData.wdes = experiment.wamData.wdes(1:experiment.frameCount,:);
    experiment.wamData.wddes = experiment.wamData.wddes(1:experiment.frameCount,:);
    experiment.wamData.tau = experiment.wamData.tau(1:experiment.frameCount,:);
    
    % truncate the WSGForce to the frameCount
    experiment.wamData.WSGForce = experiment.wamData.WSGForce(1:experiment.frameCount,:);
    
    % truncate trackData to frameCount
    experiment.trackData.frameUpdated = experiment.trackData.frameUpdated(1:experiment.frameCount);
    experiment.trackData.rawMarkers = experiment.trackData.rawMarkers(:,:,1:experiment.frameCount);
    
    % determine number of trackables
    trackableCount = length(experiment.trackData.trackables);
    
    % truncate trackables to frameCount
    for t=1:trackableCount
        % truncate lastTrack to frameCount
        experiment.trackData.trackables(t).lastTrack = experiment.trackData.trackables(t).lastTrack(1:experiment.frameCount);
        
        % truncate merr,mapos,mepos to frameCount
        experiment.trackData.trackables(t).merr = experiment.trackData.trackables(t).merr(1:experiment.frameCount,:);
        experiment.trackData.trackables(t).mapos = experiment.trackData.trackables(t).mapos(:,:,1:experiment.frameCount);
        experiment.trackData.trackables(t).mepos = experiment.trackData.trackables(t).mepos(:,:,1:experiment.frameCount);
        
        % truncate tpos,tqtr to frameCount
        experiment.trackData.trackables(t).tpos = experiment.trackData.trackables(t).tpos(1:experiment.frameCount,:);
        experiment.trackData.trackables(t).tqtr = experiment.trackData.trackables(t).tqtr(1:experiment.frameCount,:);
    end
    
    % make the frameTimes relative to the start of the experiment
    if(experiment.frameCount >= 1)
        experiment.frameTimes = experiment.frameTimes-experiment.frameTimes(1);
    end
    
    % compute overall frame rate
    if(experiment.frameCount >= 2)
        avgFrameRate = 1/mean(experiment.frameTimes(2:end)-experiment.frameTimes(1:end-1));
    else
        avgFrameRate = nan;
    end
    
    % compute frame rate for cameras
    frameTimesCam = experiment.frameTimes(experiment.trackData.frameUpdated == 1);
    if(length(frameTimesCam) >= 2)
        avgFrameRateCam = 1/mean(frameTimesCam(2:end)-frameTimesCam(1:end-1));
    else
        avgFrameRateCam = nan;
    end
    
    % clear the temporary variables for experiment
    experiment = rmfield(experiment,{'state' 'record'});
end
