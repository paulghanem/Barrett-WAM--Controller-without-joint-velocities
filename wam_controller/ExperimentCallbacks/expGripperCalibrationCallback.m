function [experiment,callbackData] = expGripperCalibrationCallback( experiment, callbackData )
%EXPGRIPPERCALIBRATIONCALLBACK Runs the gripper calibration.
%   Finds the last D value for the DH parameters for the arm
    
    arm_ready = callbackData.wamInfo.arm.ready;
    arm_home_norest = callbackData.wamInfo.arm.home_norest;
        
    switch(experiment.state)
       case 1
       % locate the cylinder in the list of trackables
            if(~isfield(experiment,'objId'))
                experiment.objId = getTrackableIdByName(experiment.trackData.trackables, 'Gripper');

                % check if cylinder exists
                if(experiment.objId == 0)
                    error('There is no trackable named "Gripper"');
                end
                
                experiment.locIndex = 1;
                experiment.locs = zeros(4,4,callbackData.inputParams.numSteps^2);
                
            end 
            
            sendArmJspaceMove(callbackData.netInfo,arm_ready);
            experiment.state = experiment.state + 1;
            
        case 2
            % Wait until arm is in ready position
            if(sum(callbackData.wamDataRcvd.qdes == arm_ready) == 7)
                % Move to first test position
                experiment.totalSteps = callbackData.inputParams.numSteps;
                experiment.stepQ = [arm_ready(1:4) getThetaStep5(experiment.totalSteps,1) getThetaStep6(experiment.totalSteps,1) 0];
                experiment.q5Step = 1;
                experiment.q6Step = 1;
                sendArmJspaceMove(callbackData.netInfo,experiment.stepQ);
                experiment.state = experiment.state + 1;
                experiment.stateTime = callbackData.recvTime;
                fprintf('State %d\n',experiment.state);
            end
            
        case 3
            
            if(sum(callbackData.wamDataRcvd.qdes == experiment.stepQ) == 7)
               
                fprintf('Taking Reading Here %d %d\n', experiment.q5Step, experiment.q6Step);
                TWAMGripper = callbackData.trackDataRcvd.trackables(experiment.objId).Tobjwam;
                experiment.locs(:,:,experiment.locIndex) = TWAMGripper;
                experiment.locIndex = experiment.locIndex + 1;
                
                
                if(experiment.q5Step < experiment.totalSteps)
                    if(experiment.q6Step < experiment.totalSteps)
                        experiment.q6Step = experiment.q6Step + 1;
                    else
                        experiment.q5Step = experiment.q5Step + 1;
                        experiment.q6Step = 1;
                    end
                    experiment.stepQ = [arm_ready(1:4) getThetaStep5(experiment.totalSteps,experiment.q5Step) getThetaStep6(experiment.totalSteps,experiment.q6Step) 0];
                    sendArmJspaceMove(callbackData.netInfo,experiment.stepQ);
                else
                    sendArmJspaceMove(callbackData.netInfo,arm_ready);
                    experiment.state = experiment.state + 1;
                    fprintf('State %d\n',experiment.state);
                end
            end
            
            
        case 4
            if(sum(callbackData.wamDataRcvd.qdes == arm_ready) == 7)
               sendArmJspaceMove(callbackData.netInfo,arm_home_norest);
               experiment.state = experiment.state + 1;
               fprintf('State %d\n',experiment.state);
            end
            
        case 5
            if(sum(callbackData.wamDataRcvd.qdes == arm_home_norest) == 7)
               
               pts = squeeze(experiment.locs(1:3,4,1:experiment.locIndex-1));
               pts = pts';
               
               figure(1);
               scatter3(pts(:,1),pts(:,2),pts(:,3));
               
               [c r res] = spherefit(pts);
               
               experiment.center = c;
               experiment.r = r;
               experiment.res = res;

               experiment.state = 0;
            end
            
    end
end

function q = getThetaStep5(numSteps,i)
    %-4.79, 1.27;
    l = linspace(-pi,0,numSteps);
    q = l(i);
end

function q = getThetaStep6(numSteps,i)
    %-1.57, 1.57;
    l = linspace(-1.5, 1.5, numSteps);
    q = l(i);
end


function [center,radius,residuals] = spherefit(x,y,z)
    %SPHEREFIT find least squares sphere
    %
    % Fit a sphere to a set of xyz data points
    % [center,radius,residuals] = shperefit(X)
    % [center,radius,residuals] = spherefit(x,y,z);
    % Input
    % x,y,z Cartesian data, n x 3 matrix or three vectors (n x 1 or 1 x n)
    % Output
    % center least squares sphere center coordinates, == [xc yc zc]
    % radius radius of curvature
    % residuals residuals in the radial direction
    %
    % Fit the equation of a sphere in Cartesian coordinates to a set of xyz
    % data points by solving the overdetermined system of normal equations,
    % ie, x^2 + y^2 + z^2 + a*x + b*y + c*z + d = 0
    % The least squares sphere has radius R = sqrt((a^2+b^2+c^2)/4-d) and
    % center coordinates (x,y,z) = (-a/2,-b/2,-c/2)

    error(nargchk(1,3,nargin)); % check input arguments
    if nargin == 1 % n x 3 matrix
       if size(x,2) ~= 3
          error ('input data must have three columns')
       else
          z = x(:,3); % save columns as x,y,z vectors
          y = x(:,2);
          x = x(:,1);
       end
    elseif nargin == 3 % three x,y,z vectors
       x = x(:); % force into columns
       y = y(:);
       z = z(:);
       if ~isequal(length(x),length(y),length(z)) % same length ?
          error('input vectors must be same length');
       end
    else % must have one or three inputs
       error('invalid input, n x 3 matrix or 3 n x 1 vectors expected');
    end

    % need four or more data points
    if length(x) < 4
       error('must have at least four points to fit a unique sphere');
    end

    % solve linear system of normal equations
    A = [x y z ones(size(x))];
    b = -(x.^2 + y.^2 + z.^2);
    a = A \ b;

    % return center coordinates and sphere radius
    center = -a(1:3)./2;
    radius = sqrt(sum(center.^2)-a(4));

    % calculate residuals
    if nargout > 2
       residuals = radius - sqrt(sum(bsxfun(@minus,[x y z],center.').^2,2));
    end
end
