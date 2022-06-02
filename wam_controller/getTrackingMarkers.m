function frameRawMarkers = getTrackingMarkers(Tcamwam)
    % update raw markers
    rawMarkerCount = calllib('NPTrackingTools', 'TT_FrameMarkerCount');

    % frameRawMarkers is the raw markers visible in this frame
    frameRawMarkers = nan(rawMarkerCount, 3);
    
    % go through all raw markers in this frame
    for i=1:rawMarkerCount
        % to turn into right handed system, we need to flip the z-axis
        x = calllib('NPTrackingTools', 'TT_FrameMarkerX',i-1);
        y = calllib('NPTrackingTools', 'TT_FrameMarkerY',i-1);
        z = calllib('NPTrackingTools', 'TT_FrameMarkerZ',i-1);
        
        % transform into wam coordinate frame
        pcam = [x y -z];
        pwam = (Tcamwam(1:3,:)*[pcam 1]')';
        
        % store the result
        frameRawMarkers(i,:) = pwam;
    end
end
