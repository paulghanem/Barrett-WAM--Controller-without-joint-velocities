function [T,errMean,errMax] = getTransformCamWamLS(calibration,wamInfo)
    % determine how many configs we are going to and how many markers there are
    numCalibSteps = size(calibration.q,1)/2;
    numCalibMarkers = size(calibration.trackedMarkers,1);
    
    % setup the function and initial condition
    x0 = zeros(6+3*numCalibMarkers,1);
    resFunc = @(x) getTransformResidualLS(x,calibration,wamInfo);
    
    % do the non-linear least squares estimate
    % we use evalc and the output parameter to suppress the result displaying to the command window
    [output,x,resnorm,residual] = evalc('lsqnonlin(resFunc,x0)');
    
    % extract the position and orientation info
    trans = x(1:3);
    ypr = x(4:6);
    
    % form the transform matrix
    R = getEuler2Rot(ypr);
    T = [R trans; 0 0 0 1];
    
    % residual vector contains errors
%     meanerr = mean(abs(residual(7:end)));
    residual = reshape(residual,3,numCalibSteps*(numCalibMarkers+1));
    resDists = sqrt(sum(residual.^2));
    errMean = mean(resDists);
    errMax = max(resDists);
end
