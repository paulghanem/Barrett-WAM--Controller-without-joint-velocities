function currThetaStep = getCalibrationCurrTheta(currStage,currStep,numCalibSteps)
    % function handle to compute current theta (we use this twice so best not to repeat the code)
    currTheta = @(step,nsteps) (-pi/2+pi*(step-1)/(nsteps-1));
    
    % determine the current desired location
    if (currStage == 1)
        currThetaStep = [currTheta(currStep,numCalibSteps) -pi/2 0 0 0 0 0];
    elseif(currStage == 2)
        currThetaStep = [0 currTheta(currStep,numCalibSteps) 0 0 0 0 0];
    else
        currThetaStep = nan(1,7);
    end
end
