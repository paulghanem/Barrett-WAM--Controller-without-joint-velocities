% returns 0 on failure, 1 on complete
function [calibration] = runCalibration(calibration, wamDataRcvd, trackDataRcvd, pauseTime, wamInfo, netInfo)
    % determine how many configs we are going to and how many markers there are
    numCalibSteps = size(calibration.q,1)/2;
    numCalibMarkers = size(calibration.trackedMarkers,1);
    
    % start towards the first calibration step
    if(calibration.stage == 0)
        calibration.stage = 1;
        firstTheta = getCalibrationCurrTheta(calibration.stage,calibration.step,numCalibSteps);
        sendArmJspaceMove(netInfo,firstTheta,.5*ones(1,7),.5*ones(1,7));
        return;
    end
    
    % check if we're done calibrating and at the home position yet
    if (calibration.stage == 3)
        if (sum(wamDataRcvd.qdes == wamInfo.arm.home_norest) == 7)
            calibration.stage = 4;
        end
        return;
    end

    % only do something if we have camera data
    if(~isempty(trackDataRcvd))
        numMarkers = size(trackDataRcvd.rawMarkers,1);
        
        % got the correct number of markers, so order them correctly
        if(numMarkers == numCalibMarkers)
            % we haven't got any readings yet
            if(isnan(calibration.trackedMarkers(1,1)))
                calibration.trackedMarkers = trackDataRcvd.rawMarkers;
            % we have previous readings, so track which markers are which
            else
                % the markers minimum distance away from each other are considered the same
                [sdist,sidx] = min(dist(trackDataRcvd.rawMarkers, calibration.trackedMarkers'));
                calibration.trackedMarkers = trackDataRcvd.rawMarkers(sidx(1,:),:);
            end
        end
        
        % determine the current desired location
        currThetaStep = getCalibrationCurrTheta(calibration.stage, calibration.step, numCalibSteps);
        
        % check if wam is at the next calibration position
        if(sum(wamDataRcvd.qdes == currThetaStep) == 7)
            currTime = java.lang.System.nanoTime()/1e9;
            if(isnan(calibration.waitStart))
                calibration.waitStart = currTime;
                return;
            elseif(currTime-calibration.waitStart < pauseTime)
                return;
            end
            
%             fprintf('got to step %d, num visible markers = %d\n', calibration.step, size(trackDataRcvd.rawMarkers,1));

            % we failed to find the right number of markers
            if (numMarkers ~= numCalibMarkers)
                calibration.markerFailures = calibration.markerFailures+1;
                return;
            end
                
            % find the one that traveled the farthest arc going to step 2 of stage 1
            % that is the one attached to the 3rd link
            if(calibration.stage == 1 && calibration.step == 2)
                mdist = dist(calibration.trackedMarkers,calibration.pcam(1:numCalibMarkers,:)');
                mdiag = nan(1,numCalibMarkers);
                for i=1:numCalibMarkers
                    mdiag(i) = mdist(i,i);
                end
                [maxdist,maxidx] = max(mdiag);
                calibration.marker3 = maxidx;
            end

            % save the pcam matrix
            if(calibration.stage == 1)
                startIdx = numCalibMarkers*(calibration.step-1)+1;
                endIdx = startIdx+numCalibMarkers-1;
                calibration.pcam(startIdx:endIdx,:) = calibration.trackedMarkers;
            elseif(calibration.stage == 2)
                startIdx = numCalibMarkers*numCalibSteps+calibration.step;
                calibration.pcam(startIdx,:) = calibration.trackedMarkers(calibration.marker3,:);
            end

            % set the calibration result for this configuration
            calibration.q(numCalibSteps*(calibration.stage-1)+calibration.step,:) = wamDataRcvd.q;

            % move to the next step in the calibration
            calibration.waitStart = nan;
            calibration.markerFailures = 0;
            calibration.step = calibration.step+1;
            if(calibration.step == numCalibSteps+1)
                calibration.stage = calibration.stage+1;
                calibration.step = 1;
            end
            
            % if all calibration steps are complete, return the arm to home
            if(calibration.stage == 3)
                sendArmHome(netInfo,wamInfo);
            else
                % determine the next desired location
                nextThetaStep = getCalibrationCurrTheta(calibration.stage, calibration.step, numCalibSteps);
                
                % move to that location
                sendArmJspaceMove(netInfo,nextThetaStep,.5*ones(1,7),.5*ones(1,7));
            end
        end
    end
end
