% recvTime, wamDataRcvd, trackDataRcvd
function experiment = runSaveFrameData(experiment, callbackData, frameBuffSize, markerBuffSize)
    % update frameCount and frameTimes
    experiment.frameCount = experiment.frameCount + 1;
    
    % determine number of trackables
    trackableCount = length(experiment.trackData.trackables);
    
    % check if we're about to overflow the data buffers
    % need to grow the buffers each by frameBuffSize (slow but necessary)
    % we compare to wamData.q here but could compare to anything really
    if(experiment.frameCount > size(experiment.wamData.q, 1))
        % grow frameTimes
        experiment.frameTimes = [experiment.frameTimes; nan(frameBuffSize,1)];
        
        % grow qdes, qddes, wdes, wddes, q, qd, tau
        experiment.wamData.q = [experiment.wamData.q; nan(frameBuffSize, 7)];
        experiment.wamData.qd = [experiment.wamData.qd; nan(frameBuffSize, 7)];
        experiment.wamData.qdes = [experiment.wamData.qdes; nan(frameBuffSize, 7)];
        experiment.wamData.qddes = [experiment.wamData.qddes; nan(frameBuffSize, 7)];
        experiment.wamData.wdes = [experiment.wamData.wdes; nan(frameBuffSize, 7)];
        experiment.wamData.wddes = [experiment.wamData.wddes; nan(frameBuffSize, 7)];
        experiment.wamData.tau = [experiment.wamData.tau; nan(frameBuffSize, 7)];
        
        % grow WSGForce
        experiment.wamData.WSGForce = [experiment.wamData.WSGForce; nan(frameBuffSize,1)];
        
        % grow frameUpdated,rawMarkers
        rawMarkerCount = size(experiment.trackData.rawMarkers,1);
        experiment.trackData.frameUpdated = [experiment.trackData.frameUpdated; false(frameBuffSize,1)];
        experiment.trackData.rawMarkers = cat(3, experiment.trackData.rawMarkers, nan(rawMarkerCount,3,frameBuffSize));
        
        % iterate through trackables
        for t=1:trackableCount
            % grow lastTrack
            experiment.trackData.trackables(t).lastTrack = [experiment.trackData.trackables(t).lastTrack; nan(frameBuffSize,1)];
            
            % grow merr,mapos,mepos
            markerCount = size(experiment.trackData.trackables(t).mbase, 1);
            experiment.trackData.trackables(t).merr = [experiment.trackData.trackables(t).merr; nan(frameBuffSize, markerCount)];
            experiment.trackData.trackables(t).mapos = cat(3, experiment.trackData.trackables(t).mapos, nan(markerCount,3,frameBuffSize));
            experiment.trackData.trackables(t).mepos = cat(3, experiment.trackData.trackables(t).mepos, nan(markerCount,3,frameBuffSize));
            
            % grow tpos,tqtr
            experiment.trackData.trackables(t).tpos = [experiment.trackData.trackables(t).tpos; nan(frameBuffSize,3)];
            experiment.trackData.trackables(t).tqtr = [experiment.trackData.trackables(t).tqtr; nan(frameBuffSize,4)];
        end
    end
    
    % save frameTimes to the experiment struct
    experiment.frameTimes(experiment.frameCount) = callbackData.recvTime;
    
    % save wam data to the experiment struct
    experiment.wamData.q(experiment.frameCount,:) = callbackData.wamDataRcvd.q;
    experiment.wamData.qd(experiment.frameCount,:) = callbackData.wamDataRcvd.qd;
    experiment.wamData.qdes(experiment.frameCount,:) = callbackData.wamDataRcvd.qdes;
    experiment.wamData.qddes(experiment.frameCount,:) = callbackData.wamDataRcvd.qddes;
    experiment.wamData.wdes(experiment.frameCount,:) = callbackData.wamDataRcvd.wdes;
    experiment.wamData.wddes(experiment.frameCount,:) = callbackData.wamDataRcvd.wddes;
    experiment.wamData.tau(experiment.frameCount,:) = callbackData.wamDataRcvd.tau;
    
    % save the WSG force to the experiment struct
    experiment.wamData.WSGForce(experiment.frameCount,:) = callbackData.wamDataRcvd.WSGForce;
    
    % no tracking data was received on this frame
    % copy data from the previous frame instead
    if (isempty(callbackData.trackDataRcvd))
        % update frameUpdated
        experiment.trackData.frameUpdated(experiment.frameCount) = 0;
        
        % copy rawMarkers
        experiment.trackData.rawMarkers(:,:,experiment.frameCount) = experiment.trackData.rawMarkers(:,:,experiment.frameCount-1);
        
        % iterate through trackables
        for t=1:trackableCount
            % copy lastTrack
            experiment.trackData.trackables(t).lastTrack(experiment.frameCount) = experiment.trackData.trackables(t).lastTrack(experiment.frameCount-1);
            
            % copy merr,mapos,mepos
            experiment.trackData.trackables(t).merr(experiment.frameCount,:) = experiment.trackData.trackables(t).merr(experiment.frameCount-1,:);
            experiment.trackData.trackables(t).mapos(:,:,experiment.frameCount) = experiment.trackData.trackables(t).mapos(:,:,experiment.frameCount-1);
            experiment.trackData.trackables(t).mepos(:,:,experiment.frameCount) = experiment.trackData.trackables(t).mepos(:,:,experiment.frameCount-1);
            
            % copy tpos,tqtr
            experiment.trackData.trackables(t).tpos(experiment.frameCount,:) = experiment.trackData.trackables(t).tpos(experiment.frameCount-1,:);
            experiment.trackData.trackables(t).tqtr(experiment.frameCount,:) = experiment.trackData.trackables(t).tqtr(experiment.frameCount-1,:);
        end
    % we got track data on this frame
    else
        % save frameUpdated
        experiment.trackData.frameUpdated(experiment.frameCount) = 1;
        
        % save rawMarkers
        rawMarkersCountFrame = size(callbackData.trackDataRcvd.rawMarkers,1);
        rawMarkersCountTotal = size(experiment.trackData.rawMarkers,1);

        % check if we're about to overflow the rawMarker buffer
        if (rawMarkersCountFrame > rawMarkersCountTotal)
            % add more markers so that we have a multiple of markerBuffSize
            numMarkersAdd = (rawMarkersCountFrame-rawMarkersCountTotal)+(markerBuffSize-mod(rawMarkersCountFrame,markerBuffSize));
            frameSizeMarkers = size(experiment.trackData.rawMarkers, 3);
            experiment.trackData.rawMarkers = cat(1, experiment.trackData.rawMarkers, nan(numMarkersAdd,3,frameSizeMarkers));
        end

        % tack on the markers for this frame
        experiment.trackData.rawMarkers(1:rawMarkersCountFrame,:,experiment.frameCount) = callbackData.trackDataRcvd.rawMarkers;
        
        % iterate through trackables
        for t=1:trackableCount
            % update lastTrack
            if(callbackData.trackDataRcvd.trackables(t).tracked)
                experiment.trackData.trackables(t).lastTrack(experiment.frameCount) = 0;
            else
                if(experiment.frameCount == 1)
                    experiment.trackData.trackables(t).lastTrack(experiment.frameCount) = 1;
                else
                    experiment.trackData.trackables(t).lastTrack(experiment.frameCount) = experiment.trackData.trackables(t).lastTrack(experiment.frameCount-1)+1;
                end
            end
            
            % update merr,mapos,mepos
            experiment.trackData.trackables(t).merr(experiment.frameCount,:) = callbackData.trackDataRcvd.trackables(t).merr;
            experiment.trackData.trackables(t).mapos(:,:,experiment.frameCount) = callbackData.trackDataRcvd.trackables(t).mapos;
            experiment.trackData.trackables(t).mepos(:,:,experiment.frameCount) = callbackData.trackDataRcvd.trackables(t).mepos;
            
            % update tpos,tqtr
            experiment.trackData.trackables(t).tpos(experiment.frameCount,:) = callbackData.trackDataRcvd.trackables(t).tpos;
            experiment.trackData.trackables(t).tqtr(experiment.frameCount,:) = callbackData.trackDataRcvd.trackables(t).tqtr;
        end
    end
end
