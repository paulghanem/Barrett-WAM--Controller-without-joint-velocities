function trackables = getTrackingTrackables(experiment,rawMarkers,Tcamwam)
    % determine number of trackables
    trackableCount = length(experiment.trackData.trackables);
    trackables = struct([]);
    
    % setup dummy variables, need these to send into function below
    [x,y,z,qx,qy,qz,qw,yaw,pitch,roll] = deal(0);
    
    % get data for each trackable
    for t=1:trackableCount
        tidx = t-1;
        
        % determine number of markers
        markerCount = size(experiment.trackData.trackables(t).mbase, 1);
            
        % check if trackable is tracked on this frame
        if(calllib('NPTrackingTools', 'TT_IsTrackableTracked', tidx))
            % set tracked
            trackables(t).tracked = 1;
            
            % the camera system uses quaternions internally, then for "convenience" gives yaw/pitch/roll
            % in XZY order, where roll=x pitch=z and yaw=y (ignore Euler angles, they are ugly to work with!!)
            
            % get trackable position/orientation
            [x, y, z, qx, qy, qz, qw, yaw, pitch, roll] = calllib('NPTrackingTools', 'TT_TrackableLocation', tidx, x, y, z, qx, qy, qz, qw, yaw, pitch, roll);
            
            % to turn into right handed system, we need to flip the z-axis
            % (any would do, but we choose to use z throughout the code)
            tposcam = [x y -z];
            
            % compute the quaternion of the object
            % we need to flip the z-axis (if you picture the quaternion in axis-angle representation)
            tqtrcam = [qx qy -qz qw];
            
            % compute Robjcam from the quaternion
            Robjcam = getQuaternion2Rot(tqtrcam);
            
            % compute transform from frame attached to object to the camera base frame
            % then transform into the wam's coordinate frame
            Tobjcam = [Robjcam tposcam'; 0 0 0 1];
            Tobjwam = double(Tcamwam*Tobjcam);
            
            % set tpos,tqtr, Tobjwam
            trackables(t).tpos = Tobjwam(1:3,4)';
            trackables(t).tqtr = getRot2Quaternion(Tobjwam(1:3,1:3));
            trackables(t).Tobjwam = Tobjwam;
            
            % compute the expected position (mepos) using mbase
            % mbase contains the marker locations in the object frame
            trackables(t).mepos = (Tobjwam(1:3,:)*[experiment.trackData.trackables(t).mbase'; ones(1,markerCount)])';
            
            % compute the minimum distance from mepos to all raw markers, then sort
            dists = dist(rawMarkers, trackables(t).mepos');
            [sdists,sidxs] = sort(dists);
            
            % mapos is the actual location of the closest marker to mepos
            % merr is the distance between mapos, mepos
            trackables(t).mapos = rawMarkers(sidxs(1,:),:);
            trackables(t).merr = sdists(1,:);
        else
            % set tracked
            trackables(t).tracked = 0;

            % set tpos,tqtr
            trackables(t).tpos = nan(1,3);
            trackables(t).tqtr = nan(1,4);
            
            % set merr,mapos,mepos
            trackables(t).merr = nan(1,markerCount);
            trackables(t).mapos = nan(markerCount,3);
            trackables(t).mepos = nan(markerCount,3);
        end
    end
end
