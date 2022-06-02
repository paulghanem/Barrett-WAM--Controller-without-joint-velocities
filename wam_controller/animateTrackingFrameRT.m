W% don't think this is quite finished
function [fig] = animateTrackingFrameRT(experiment,callbackData)
    % create a dummy experiment with the data needed to animate
    expDummy = struct('trackData',callbackData.trackDataRcvd,'wamData',callbackData.wamDataRcvd);

    % copy over the trackable names so we can animate what's happening
    for tidx=1:length(experiment.trackData.trackables)
        % replace tracked field with lastTrack
        expDummy.trackData.trackables(tidx).lastTrack = ~logical(callbackData.trackDataRcvd.trackables(tidx).tracked);
        
        % add the trackable name and mbase fields
        expDummy.trackData.trackables(tidx).name = experiment.trackData.trackables(tidx).name;
        expDummy.trackData.trackables(tidx).mbase = experiment.trackData.trackables(tidx).mbase;
    end
    
    % remove the tracked field which is no longer needed
    expDummy.trackData.trackables = rmfield(expDummy.trackData.trackables,'tracked');
    
    % now animate using the dummy experiment
    fig = animateTrackingFrame(expDummy,1,1,0);
end
