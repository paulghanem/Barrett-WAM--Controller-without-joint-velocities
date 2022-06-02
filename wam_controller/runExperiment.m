function [experiment, tformInfo, wamInfo, calibration] = runExperiment(ttProjectFile, callbackFunc, inputParams, tformInfo)
    if (~ischar(callbackFunc))
        error('callbackFunc must be a function name of type char');
    end
    
    % setup constants
    FRAME_BUFF_SIZE = 1000;
    MARKER_BUFF_SIZE = 10;
    SHIFT_ACTIVE_TIMEOUT = 30;
    UDP_RECEIVE_TIMEOUT = 2;
    
    CALIB_PAUSE_TIME = 0;
    MAX_CALIB_FAILURES = 10;
    NUM_CALIB_MARKERS = 5; % 1 of these is assumed to be on link 3, the rest on link 1
    NUM_CALIB_STEPS = 9;
    TRANSFORM_MAX_ERROR = 0.002; % 0.1
    
    % initialize UDP communication
    fprintf('\n');
    fprintf('Initializing UDP Communication...\n');
    [netInfo] = initUdpCommunication();
    
    % initialize tracking system
    fprintf('Initializing Tracking System...\n');
    [trackData] = initTrackingSystem(ttProjectFile, FRAME_BUFF_SIZE, MARKER_BUFF_SIZE);
    
    % initialize wam info struct
    fprintf('Initializing WAM Info Struct...\n');
    [wamInfo] = initWamInfo();
    
    % initialize udp/wam stuff
    fprintf('Initializing WAM Controller...\n');
    [wamData] = initWamControl(FRAME_BUFF_SIZE);

    % now setup the actual experiment
    experiment = struct('state',1,'record',0,'handedness','R','frameTimes',nan(FRAME_BUFF_SIZE,1),'frameCount',0,'wamData',wamData,'trackData',trackData);
    fprintf('Connecting to GPE Server...\n');
    con = connect_to_GPE();
    experiment.Connection = con;
    firstPacket = 1;
    haveRecorded = 0;  % whether we have recorded at least one frame yet
    blockNum = 0;
    timeouts = 0;
    
    
    % if something screws up, shutdown the system gracefully
    c = onCleanup(@()runCleanupGracefully(netInfo,con));
    
    % check whether input params were provided
    if (nargin < 3)
        inputParams = struct([]);
    end
    
    % check whether we are provided calibration data already
    if (nargin < 4)
        % calibration was not given as input
        calibDone = 0;
        
        % setup the calibration parameters
        tformInfo = struct('Tcamwam',eye(4),'errMean',nan,'errMax',nan);
        calibration = struct('stage',0,'step',1,'waitStart',nan,'markerFailures',0,'q',nan(2*NUM_CALIB_STEPS,7),'marker3',nan, ...
            'pcam',nan(NUM_CALIB_STEPS*(NUM_CALIB_MARKERS+1),3),'trackedMarkers',nan(NUM_CALIB_MARKERS,3));
    else
        % calibration was given as input
        calibDone = 1;
        calibration = struct([]);
    end
    
    % packets will not come from xPC target until WAM is Shift-Activated
    fprintf('\nWaiting for user to Shift-Activate the WAM...\n\n');
    
    % loop until the experiment is over
    while(experiment.state ~= 0)
        
        % wait 30 seconds for the WAM to be Shift-Activated
        if (firstPacket && timeouts >= SHIFT_ACTIVE_TIMEOUT)
            break;
        % WAM should not take longer than 2 seconds to send next UDP packet
        elseif (~firstPacket && timeouts >= UDP_RECEIVE_TIMEOUT)
            error('UDP receive timed out. xPC target may have disconnected or WAM Shift-Idled.');
        end
        
        % try receiving UDP, catch java SocketException if it times out
        try
            % receive feedback packet from xPC target
            [wamDataRcvd, recvTime, blockNumRcvd] = getWamPacket(netInfo);
            timeouts = 0;
        catch exc
            % check the exception type
            if(strcmp(exc.identifier,'MATLAB:Java:GenericException'))
                timeouts = timeouts + 1;
                continue;
            % the socket didn't time out, it was some other error
            else
                rethrow(exc);
            end
        end
        
         % get tracking frame from the camera system
        [trackDataRcvd] = getTrackingFrame(experiment,tformInfo.Tcamwam,calibDone&&~haveRecorded);
        
        % check if we received a new packet
        if (blockNumRcvd <= blockNum)
            continue;
        end
        
        % update blockNum
        blockNum = blockNumRcvd;
        
        % do calibration if we haven't already
        if(~calibDone)
            if(calibration.stage == 0)
                fprintf('Calibration Started Successfully\n');
            end
            
            % run through the calibration to collect marker data
            [calibration] = runCalibration(calibration, wamDataRcvd, trackDataRcvd, CALIB_PAUSE_TIME, wamInfo, netInfo);
            
            % but there were not 4 visible markers for MAX_CALIB_FAILURES frames in a row
            if (calibration.markerFailures > MAX_CALIB_FAILURES)
                error('Wrong number of markers visible during calibration (%d)', size(trackDataRcvd.rawMarkers,1));
            end
            
            % the calibration is complete
            if(calibration.stage == 4)
                % compute the transform and check within tolerance
                [Tcamwam, errMean, errMax] = getTransformCamWamLS(calibration,wamInfo);
                tformInfo.Tcamwam = Tcamwam;
                tformInfo.errMean = errMean;
                tformInfo.errMax = errMax;
                
                % check that we're within tolerance
                if(errMean < TRANSFORM_MAX_ERROR)
                    calibDone = 1;
                    fprintf('Calibration Completed Successfully (mean error = %.4f m)\n\n', errMean);
                else
                    error('Tcamwam was not found within tolerance (%.4f > %.4f)', errMean, TRANSFORM_MAX_ERROR);
                end
            end
            
            % don't save data, it's in the camera frame
            % the next time through we will use Tcamwam
            continue;
        end
        
        % the WAM was Shift-Activated and we finished calibration
        if (firstPacket == 1)
            setWSGSetForcePeriod(netInfo,-1);
            pause(1);
            setWSGPositionPeriod(netInfo,50);
            firstPacket = 0;
            sendStartTorque(netInfo);
            fprintf('Experiment Started Successfully\n');
        end
        
        % send experiment data to function callback
        % experiment.state will be set to 0 if experiment is done
        % we form callbackData into a struct so we can expand on it later if needed
        % without changing the form of the callback function
        callbackData = struct('wamDataRcvd',wamDataRcvd,'trackDataRcvd',trackDataRcvd,'recvTime',recvTime,'netInfo',netInfo,'wamInfo',wamInfo,'inputParams',inputParams);
        [experiment,callbackData] = feval(callbackFunc, experiment, callbackData);
        
        % save the data to the experiment struct, if we're set to record
        if(experiment.record)
            haveRecorded = 1;
            [experiment] = runSaveFrameData(experiment, callbackData, FRAME_BUFF_SIZE, MARKER_BUFF_SIZE);
        end
    end
    
    if(firstPacket == 1)
        fprintf('Experiment Failed to Start\n');
    else
        % cleanup all the data
        [experiment, avgFrameRate, avgFrameRateCam] = runCleanupData(experiment);
        fprintf('Experiment Completed Successfully\n');
        if(experiment.frameCount == 0)
            fprintf('    No data was recorded. If you want to record data, be sure to explicitly\n');
            fprintf('    set the experiment.record flag in your callback function.\n');
        else
            fprintf('    %d frames of data were recorded.\n\n', experiment.frameCount);
            fprintf('    Total Recording Time: %.2f s\n', experiment.frameTimes(end)-experiment.frameTimes(1));
            fprintf('    Average Framerate: %.2f\n', avgFrameRate);
            fprintf('    Average Framerate (cameras): %.2f\n', avgFrameRateCam);
        end
        fprintf('\n');
    end
end

function Connection = connect_to_GPE()
    %% Connect to GPE
    %IP = '128.213.17.141';
    IP = '128.213.17.85';
    Port = 18001;
    Connection = pnet('tcpconnect',IP,Port);
    if Connection < 0
        fprintf('\nFailed to establish connection with GPE\n')
    else
        fprintf('\nConnection established (IP: %s, Port %d)\n',IP,Port)
    end
end
