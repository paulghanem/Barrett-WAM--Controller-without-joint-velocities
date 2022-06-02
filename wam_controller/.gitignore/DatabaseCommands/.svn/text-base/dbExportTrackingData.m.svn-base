% export the experiment to the database with a description
% returns the experimentId so we can go back and find the experiment in the database
function [experimentId] = dbExportTrackingData(experiment,tformInfo,description)
    experimentId = -1;
    
    % check if the user really wants to export
    fprintf('\n');
    yn = upper(input('Really export data to database? Y/N [N]: ', 's'));
    if (length(yn) ~= 1 || (yn ~= 'Y' && yn ~= 'N'))
        fprintf('\nInvalid input. Answer must be Y or N (caps insensitve). Exiting now.\n\n');
        return;
    end
    
    % the user changed his mind
    if(yn == 'N')
        fprintf('\nData not exported. Exiting now.\n\n');
        return;
    end
    
    fprintf('\n');
    
    % connect using JDBC (faster than ODBC :-D)
    % a matlab bug causes all global variables to be cleared
    % from the workspace when calling javaaddpath
    javaaddpath('mysql-connector-java-5.1.13-bin.jar');
    dbConn = database('Experiments', 'robotics', 'sensornet','com.mysql.jdbc.Driver','jdbc:mysql://grasp.robotics.cs.rpi.edu:3306/Experiments');
    
    % check that we're connected
    if(~isconnection(dbConn))
        error('Connection Error\n%s', dbConn.Message);
    end
    
    % if something screws up, close the connection and rollback if necessary
    c = onCleanup(@()dbCleanupGracefully(dbConn));
    
    % set AutoCommit to off, so that no bad data goes in the database if
    % the function fails for whatever reason
    set(dbConn,'AutoCommit','off');
    
    
    % determine the framecount and trackable count
    jointCount = 4;
    frameCount = experiment.frameCount;
    trackableCount = length(experiment.trackData.trackables);

    
    % EXPERIMENT INFO
    % create experiment
    fprintf('Exporting experiment metadata...\n');
    experimentData.timestamp = {datestr(now,31)};
    experimentData.type = {1};
    experimentData.description = {description};
    experimentData.handedness = {experiment.handedness};
    fastinsert(dbConn, 'experiments', {'timestamp','type','description','handedness'}, experimentData);

    % determine experiment id
    experimentId = cell2mat(fetch(dbConn, 'SELECT LAST_INSERT_ID()'));
    
    
    % CALIBRATION INFO
    % export transform info
    fprintf('Exporting calibration result...\n');
    calibrationData.experimentId = experimentId;
    calibrationData.errMean = tformInfo.errMean;
    calibrationData.errMax = tformInfo.errMax;
    calibrationData.T11 = tformInfo.Tcamwam(1,1);
    calibrationData.T12 = tformInfo.Tcamwam(1,2);
    calibrationData.T13 = tformInfo.Tcamwam(1,3);
    calibrationData.T14 = tformInfo.Tcamwam(1,4);
    calibrationData.T21 = tformInfo.Tcamwam(2,1);
    calibrationData.T22 = tformInfo.Tcamwam(2,2);
    calibrationData.T23 = tformInfo.Tcamwam(2,3);
    calibrationData.T24 = tformInfo.Tcamwam(2,4);
    calibrationData.T31 = tformInfo.Tcamwam(3,1);
    calibrationData.T32 = tformInfo.Tcamwam(3,2);
    calibrationData.T33 = tformInfo.Tcamwam(3,3);
    calibrationData.T34 = tformInfo.Tcamwam(3,4);
    fastinsert(dbConn, 'calibrations', {'experimentId','errMean','errMax','T11','T12','T13','T14','T21','T22','T23','T24','T31','T32','T33','T34'}, calibrationData);
    
    
    % FRAME INFO
    % set frame times
    fprintf('Exporting frame data...\n');
    frameData = nan(frameCount,3);
    frameData(:,1) = repmat(experimentId,frameCount,1);
    frameData(:,2) = experiment.frameTimes;
    frameData(:,3) = experiment.trackData.frameUpdated;

    %fastinsert(dbConn, 'frames', {'experimentId','frameTime'}, frameData);
    dbBulkInsert(dbConn,'frames',{'experimentId','frameTime','trackingFrameUpdated'},frameData);

    % determine frame id's
    frameIds = cell2mat(fetch(dbConn, sprintf('SELECT id FROM frames WHERE experimentId=%d ORDER BY id', experimentId)));


    % WAM INFO
    % insert the data for the wam for the entire experiment
    fprintf('Exporting WAM data...\n');
    wamData = nan(jointCount*frameCount,9);
    wamData(:,1) = reshape(repmat(frameIds,1,jointCount)', frameCount*jointCount, 1);
    wamData(:,2) = repmat((1:jointCount)', frameCount, 1);
    wamData(:,3) = reshape(experiment.wamData.q',frameCount*jointCount,1);
    wamData(:,4) = reshape(experiment.wamData.qd',frameCount*jointCount,1);
    wamData(:,5) = reshape(experiment.wamData.qdes',frameCount*jointCount,1);
    wamData(:,6) = reshape(experiment.wamData.qddes',frameCount*jointCount,1);
    wamData(:,7) = reshape(experiment.wamData.wdes',frameCount*jointCount,1);
    wamData(:,8) = reshape(experiment.wamData.wddes',frameCount*jointCount,1);
    wamData(:,9) = reshape(experiment.wamData.tau',frameCount*jointCount,1);

    dbBulkInsert(dbConn,'wamData',{'frameId','jointId','q','qd','qdes','qddes','wdes','wddes','tau'},wamData);

    % TRACKING INFO
    % insert raw marker data
    fprintf('Exporting raw marker data...\n');
    markerCount = size(experiment.trackData.rawMarkers,1);
    markerData = nan(frameCount*markerCount,4);
    markerData(:,1) = reshape(repmat(frameIds,1,markerCount)', frameCount*markerCount, 1);
    markerData(:,2) = reshape(squeeze(experiment.trackData.rawMarkers(:,1,:)), frameCount*markerCount, 1);
    markerData(:,3) = reshape(squeeze(experiment.trackData.rawMarkers(:,2,:)), frameCount*markerCount, 1);
    markerData(:,4) = reshape(squeeze(experiment.trackData.rawMarkers(:,3,:)), frameCount*markerCount, 1);

    %fastinsert(dbConn,'rawMarkers',{'frameId','markerx','markery','markerz'},markerData);
    dbBulkInsert(dbConn,'rawMarkers',{'frameId','markerx','markery','markerz'},markerData);

    fprintf('Exporting trackables (%d total)\n', trackableCount);

    % insert the data for each trackable
    for t=1:trackableCount
        trackable = experiment.trackData.trackables(t);
        fprintf('Trackable #%d (%s)\n', t, trackable.name);

        % insert trackable markers then find the associated id's
        fprintf('  Exporting marker base locations...\n');
        trackableData.experimentId = {experimentId};
        trackableData.trackableName = {trackable.name};
        fastinsert(dbConn, 'trackables', {'experimentId','trackableName'}, trackableData);

        trackableId = cell2mat(fetch(dbConn, 'SELECT LAST_INSERT_ID()'));

        markerCount = size(trackable.mbase,1);
        trackableMarkerData = nan(markerCount,4);
        trackableMarkerData(:,1) = repmat(trackableId,markerCount,1);
        trackableMarkerData(:,2:4) = trackable.mbase(:,1:3);
        dbBulkInsert(dbConn, 'trackableMarkers', {'trackableId','mbasex','mbasey','mbasez'}, trackableMarkerData);

        markerIds = cell2mat(fetch(dbConn, sprintf('SELECT id FROM trackableMarkers WHERE trackableId=%d ORDER BY id', trackableId)));

        % insert trackable frames data
        fprintf('  Exporting trackable data...\n');
        trackableFramesData = nan(frameCount,10);
        trackableFramesData(:,1) = trackableId*ones(frameCount,1);
        trackableFramesData(:,2:10) = [frameIds trackable.lastTrack trackable.tpos trackable.tqtr];

        dbBulkInsert(dbConn,'trackableFrames',{'trackableId','frameId','lastTrack','tposx','tposy','tposz','tqx','tqy','tqz','tqw'},trackableFramesData);

        % insert trackable frame markers data
        fprintf('  Exporting trackable marker data...\n');
        trackableMarkerFramesData = nan(frameCount*markerCount,9);
        trackableMarkerFramesData(:,1) = reshape(repmat(frameIds,1,markerCount)', frameCount*markerCount, 1);
        trackableMarkerFramesData(:,2) = repmat(markerIds, frameCount, 1);
        trackableMarkerFramesData(:,3) = reshape(squeeze(trackable.mapos(:,1,:)), frameCount*markerCount, 1);
        trackableMarkerFramesData(:,4) = reshape(squeeze(trackable.mapos(:,2,:)), frameCount*markerCount, 1);
        trackableMarkerFramesData(:,5) = reshape(squeeze(trackable.mapos(:,3,:)), frameCount*markerCount, 1);
        trackableMarkerFramesData(:,6) = reshape(squeeze(trackable.mepos(:,1,:)), frameCount*markerCount, 1);
        trackableMarkerFramesData(:,7) = reshape(squeeze(trackable.mepos(:,2,:)), frameCount*markerCount, 1);
        trackableMarkerFramesData(:,8) = reshape(squeeze(trackable.mepos(:,3,:)), frameCount*markerCount, 1);
        trackableMarkerFramesData(:,9) = reshape(trackable.merr',frameCount*markerCount,1);

        dbBulkInsert(dbConn,'trackableMarkerFrames',{'frameId','markerId','mactx','macty','mactz','mexpx','mexpy','mexpz','merr'},trackableMarkerFramesData);
    end
    
    % finished, commit our changes
    commit(dbConn);
    fprintf('\n');
end