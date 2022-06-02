function [trackData] = initTrackingSystem(ttProjectFile, frameBuffSize, markerBuffSize)
    % load the tracking tools library
    if ~libisloaded('NPTrackingTools')
        fprintf('\nLoading NPTrackingTools library...\n\n');
        addpath('C:\Program Files (x86)\NaturalPoint\TrackingTools\lib');
        addpath('C:\Program Files (x86)\NaturalPoint\TrackingTools\inc');

        s = warning();
        warning off all;
        [notfound,warnings] = loadlibrary('NPTrackingTools','NPTrackingTools.h');
        warning(s);

        if (~isempty(notfound))
            fprintf('\nSome functions were defined in the header file but not in the library\n');
            fprintf('  * %s\n', notfound{:});
            fprintf('\n');
        end

        if (~isempty(warnings))
            fprintf('\n');
            warning('\nReceived warnings while loading NPTrackingTools\n%s\n\n',warnings); %#ok<WNTAG>
        end
    end
    
    % initialize cameras
    calllib('NPTrackingTools', 'TT_Initialize');
    
    % load the project file which sets up cameras correctly
    if(~exist(ttProjectFile,'file'))
        error('TT project file does not exist');
    end
    calllib('NPTrackingTools', 'TT_LoadProject', ttProjectFile);
    
    [x,y,z] = deal(0);
    
    % determine number of trackables
    trackableCount = calllib('NPTrackingTools', 'TT_TrackableCount');
    
    % get trackable names and marker data
    trackableNames = cell(trackableCount,1);
    for t=1:trackableCount
        tidx = t-1;
        trackableNames{t} = calllib('NPTrackingTools', 'TT_TrackableName', tidx);
    end
    
    % init trackables struct
    trackables = struct([]);
    
    % get trackable marker data
    for t=1:trackableCount
        tidx = t-1;
        
        % determine number of markers
        markerCount = calllib('NPTrackingTools', 'TT_TrackableMarkerCount', tidx);
        
        % init empty buffers
        trackables(t).name = trackableNames{t};
        trackables(t).lastTrack = nan(frameBuffSize,1);
        trackables(t).mbase = nan(markerCount,3);
        trackables(t).merr = nan(frameBuffSize, markerCount);
        trackables(t).mapos = nan(markerCount,3,frameBuffSize);
        trackables(t).mepos = nan(markerCount,3,frameBuffSize);
        trackables(t).tpos = nan(frameBuffSize,3);
        trackables(t).tqtr = nan(frameBuffSize,4);
        
        % load the base marker locations
        for m=1:markerCount
            midx = m-1;
            % mbase is relative to the coordinate frame attached to the object
            % whose origin is initially at the centroid and Euler angles all zeros
            [x, y, z] = calllib('NPTrackingTools', 'TT_TrackableMarker', tidx, midx, x, y, z);
            trackables(t).mbase(m,:) = [x y -z];
        end
    end
    
    rawMarkers = nan(markerBuffSize,3,frameBuffSize);
    frameUpdated = false(frameBuffSize,1);
    
    trackData = struct('trackables',trackables,'rawMarkers',rawMarkers,'frameUpdated',frameUpdated);
    
    % wait for tracking system to become responsive
    update = @()calllib('NPTrackingTools', 'TT_Update');
    frameTime = @()calllib('NPTrackingTools', 'TT_FrameTimeStamp');
    
    ures = update(); ftime = frameTime();
    while(ures ~= 0 || ftime == 0)
        pause(0.001);
        ures = update();
        ftime = frameTime();
    end
    
%     pause(0.5);
%     calllib('NPTrackingTools', 'TT_Update')
%     
%     for i=1:50
%         %update = calllib('NPTrackingTools', 'TT_Update')
%         pause(.1);
%     end
end
