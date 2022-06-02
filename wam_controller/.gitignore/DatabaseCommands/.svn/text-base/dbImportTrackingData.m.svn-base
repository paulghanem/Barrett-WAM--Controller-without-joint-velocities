% import tracking data into the experiment struct
function [experiment,tformInfo,metaData] = dbImportTrackingData(experimentId)
    fprintf('\n');
    
    % connect using JDBC (faster than ODBC :-D)
    % a matlab bug causes all global variables to be cleared
    % from the workspace when calling javaaddpath
    javaaddpath('mysql-connector-java-5.1.13-bin.jar');
    dbConn = database('Experiments', 'robotics', 'sensornet','com.mysql.jdbc.Driver','jdbc:mysql://grasp.robotics:3306/Experiments');
    
    % check that we're connected
    if(~isconnection(dbConn))
        error('Connection Error\n%s', dbConn.Message);
    end
    
    % if something screws up, close the connection, no rollback is necessary
    c = onCleanup(@()dbCleanupGracefully(dbConn));
    
    % we don't need to set AutoCommit, since we're not trying to write anything
    % to the database in this script
    
    
    % EXPERIMENT INFO
    % load metadata about the experiment
    fprintf('Importing experiment metadata...\n');
    experimentData = fetch(dbConn, sprintf('SELECT timestamp,type,description,handedness FROM experiments WHERE id=%d',experimentId));
    metaData = struct('timestamp',experimentData{1},'type',experimentData{2},'description',experimentData{3});
    handedness = experimentData{4};

    
    % CALIBRATION INFO
    % load transform info
    fprintf('Importing calibration result...\n');
    calibrationData = cell2mat(fetch(dbConn, sprintf('SELECT errMean,errMax,T11,T12,T13,T14,T21,T22,T23,T24,T31,T32,T33,T34 FROM calibrations WHERE experimentId=%d',experimentId)));
    errMean = calibrationData(1,1);
    errMax = calibrationData(1,2);
    Tcamwam = [reshape(calibrationData(1,3:end),4,3)'; 0 0 0 1];
    tformInfo = struct('Tcamwam',Tcamwam,'errMean',errMean,'errMax',errMax);
    
    
    % FRAME INFO
    % load frame times
    fprintf('Importing frame data...\n');
    frameData = fetch(dbConn, sprintf('SELECT frameTime,trackingFrameUpdated FROM frames WHERE experimentId=%d ORDER BY id', experimentId));
    frameTimes = cell2mat(frameData(:,1));
    frameUpdated = cell2mat(frameData(:,2));
    frameCount = length(frameTimes);


    % WAM INFO
    fprintf('Importing WAM data...\n');
    wamData = cell2mat(fetch(dbConn, sprintf('SELECT q,qd,qdes,qddes,wdes,wddes,tau FROM wamData WHERE frameId IN (SELECT id FROM frames WHERE experimentId=%d) ORDER BY jointId,frameId',experimentId)));
    jointCount = size(wamData,1)/frameCount;
    q = reshape(wamData(:,1),frameCount,jointCount);
    qd = reshape(wamData(:,2),frameCount,jointCount);
    qdes = reshape(wamData(:,3),frameCount,jointCount);
    qddes = reshape(wamData(:,4),frameCount,jointCount);
    wdes = reshape(wamData(:,5),frameCount,jointCount);
    wddes = reshape(wamData(:,6),frameCount,jointCount);
    tau = reshape(wamData(:,7),frameCount,jointCount);


    % TRACKING INFO        
    % load raw marker data
    fprintf('Importing raw marker data...\n');
    markerCount = cell2mat(fetch(dbConn, sprintf('SELECT count(*) FROM rawMarkers WHERE frameId=(SELECT MIN(id) FROM frames WHERE experimentId=%d)',experimentId)));
    rawMarkerData = cell2mat(fetch(dbConn, sprintf('SELECT markerx,markery,markerz FROM rawMarkers WHERE frameId IN (SELECT id FROM frames WHERE experimentId=%d)',experimentId)));
    rawMarkers = permute(reshape(rawMarkerData',3,markerCount,frameCount),[2 1 3]);

    % load trackable names and id's
    trackablesData = fetch(dbConn, sprintf('SELECT id,trackableName FROM trackables WHERE experimentId=%d ORDER BY id', experimentId));
    if (isempty(trackablesData))
        trackableCount = 0;
        trackables = struct([]);
    else
        trackableIds = cell2mat(trackablesData(:,1));
        trackableCount = length(trackableIds);
    end

    fprintf('Importing trackables (%d total)\n', trackableCount);

    for t=1:trackableCount
        trackables(t).name = trackablesData{t,2};
        fprintf('Trackable #%d (%s)\n', t, trackables(t).name);

        % load mbase positions
        fprintf('  Importing marker base locations...\n');
        trackableMarkersData = fetch(dbConn, sprintf('SELECT id,mbasex,mbasey,mbasez FROM trackableMarkers WHERE trackableId=%d ORDER BY id', trackableIds(t)));
        markerIds = cell2mat(trackableMarkersData(:,1));
        trackables(t).mbase = cell2mat(trackableMarkersData(:,2:4));
        markerCount = length(markerIds);

        % load trackable frame info
        fprintf('  Importing trackable data...\n');
        trackableFramesData = fetch(dbConn, sprintf('SELECT tposx,tposy,tposz,tqx,tqy,tqz,tqw,lastTrack FROM trackableFrames WHERE trackableId=%d ORDER BY frameId', trackableIds(t)));
        trackables(t).tpos = cell2mat(trackableFramesData(:,1:3));
        trackables(t).tqtr = cell2mat(trackableFramesData(:,4:7));
        trackables(t).lastTrack = cell2mat(trackableFramesData(:,8));

        trackables(t).mapos = nan(3,3,frameCount);
        trackables(t).mepos = nan(3,3,frameCount);

        % grab the trackable marker positions
        fprintf('  Importing trackable marker data...\n');
        for m=1:markerCount
            trackableMarkerFramesData = fetch(dbConn, sprintf('SELECT mactx,macty,mactz,mexpx,mexpy,mexpz,merr FROM trackableMarkerFrames WHERE markerId=%d ORDER BY frameId',markerIds(m)));
            mapos = cell2mat(trackableMarkerFramesData(:,1:3));
            mepos = cell2mat(trackableMarkerFramesData(:,4:6));
            merr = cell2mat(trackableMarkerFramesData(:,7));
            trackables(t).mapos(m,:,:) = reshape(mapos',1,3,frameCount);
            trackables(t).mepos(m,:,:) = reshape(mepos',1,3,frameCount);
            trackables(t).merr(:,m) = merr;
        end
    end
    
    % finished
    fprintf('\n');
    
    % build the experiment struct
    wamData = struct('q',q,'qd',qd,'qdes',qdes,'qddes',qddes,'wdes',wdes,'wddes',wddes,'tau',tau);
    trackData = struct('frameUpdated',frameUpdated,'trackables',trackables,'rawMarkers',rawMarkers);
    experiment = struct('handedness',handedness,'frameCount',frameCount,'frameTimes',frameTimes,'trackData',trackData,'wamData',wamData);
end
