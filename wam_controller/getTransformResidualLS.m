function f = getTransformResidualLS(x,calibration,wamInfo)
    % determine how many configs we are going to and how many markers there are
    numCalibSteps = size(calibration.q,1)/2;
    numCalibMarkers = size(calibration.trackedMarkers,1);
    
    % extract the position and orientation info
    trans = x(1:3);
    ypr = x(4:6);
    
    % extract the guesses for all marker locations
    % numCalibMarkers markers in p1
    % 1 marker in p4
    pwam = reshape(x(7:end),3,numCalibMarkers);
    p1 = pwam(:,1:numCalibMarkers-1);
    p3 = pwam(:,numCalibMarkers);
    
    % form the transform matrix
    Rcamwam = getEuler2Rot(ypr);
    Tcamwam = [Rcamwam trans; 0 0 0 1];
    
    % initialize f
    f = zeros(3*numCalibSteps*(numCalibMarkers+1),1);

    % compute the error for the transform, for stage 1
    for stage=1:2
        for step=1:numCalibSteps
            % compute T(frame 1/3 to base) for this configuration
            q = calibration.q(numCalibSteps*(stage-1)+step,:);
            T1 = getTransformN2Base(1,wamInfo,q);
            T3 = getTransformN2Base(3,wamInfo,q);

            % compute the residuals for this config, stage 1
            if(stage == 1)
                % extract p1 and p3 points for this configuration
                startIdx = numCalibMarkers*(step-1)+1;
                endIdx = startIdx+numCalibMarkers-1;
                pcam = calibration.pcam(startIdx:endIdx,:);
                p1cam = removerows(pcam,calibration.marker3)';
                p3cam = pcam(calibration.marker3,:)';
                
                % compute the distance vectors using the transformation
                vect1 = Tcamwam(1:3,:)*[p1cam; ones(1,numCalibMarkers-1)] - T1(1:3,:)*[p1; ones(1,numCalibMarkers-1)];
                vect3 = Tcamwam(1:3,:)*[p3cam; 1] - T3(1:3,:)*[p3; 1];
                
                % now insert the error into the residual vector f
                startIdx = 3*numCalibMarkers*(step-1)+1;
                endIdx = startIdx+3*numCalibMarkers-1;
                f(startIdx:endIdx) = [reshape(vect1,3*(numCalibMarkers-1),1); reshape(vect3,3,1)];
            % compute the residuals for this config, stage 2
            else
                % extract the p3 point for this configuration
                startIdx = numCalibMarkers*numCalibSteps+step;
                p3cam = calibration.pcam(startIdx,:)';
                
                % compute the distance vectors using the transformation
                vect3 = Tcamwam(1:3,:)*[p3cam; 1] - T3(1:3,:)*[p3; 1];
                
                % now insert the error into the residual vector f
                startIdx = 3*numCalibMarkers*numCalibSteps+3*(step-1)+1;
                endIdx = startIdx+2;
                f(startIdx:endIdx) = vect3;
            end
        end
    end
end
