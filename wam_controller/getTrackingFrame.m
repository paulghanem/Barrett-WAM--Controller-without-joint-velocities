% if there was no tracking update and needData=0, this will return an empty struct
function [data] = getTrackingFrame(experiment,Tcamwam,needData)
    % update tracking system (0 means the update was successful)
    update = (calllib('NPTrackingTools', 'TT_Update')==0);
    
    % if not updated, grab the previous frames data
    % if this is the first frame, we already have data from the update at
    % the calibration, so can still grab that with the functions below
    if(~update && ~needData)
        data = struct([]);
        return;
    end
    
    
    % NOTE: all tracking tools functions return in *left* handed coordinate system!!!
    % the functions below should take care of converting into right handed system
    
    % update raw markers, trackables
    rawMarkers = getTrackingMarkers(Tcamwam);
    trackables = getTrackingTrackables(experiment,rawMarkers,Tcamwam);

    % form the struct to return
    data = struct('rawMarkers',rawMarkers,'trackables',trackables);
end
