% sets max acceleration, and max close velocity for fingers
% default ACCEL
%   Fingers: 4, Spread: 2
% default MCV
%   Fingers: 100, Spread: 60
function sendHandFingerMaxVA(netInfo, vMax, aMax)
    % check v_max and a_max
    if (~isscalar(vMax) || ~isscalar(aMax) || ~strcmp(class(vMax),'double') || ~strcmp(class(aMax),'double'))
        error('vMax and aMax must be scalar doubles');
    end
    
    % check for appropriate ranges
    if(vMax < 16)
        vMax = 16;
    elseif(vMax > 10000) %4080
        vMax = 10000;
    end
    
    if(aMax < 0)
        aMax = 0;
    elseif(aMax > 65535)
        aMax = 65535;
    end
        
    vMax = int32(vMax);
    aMax = int32(aMax);
    
    vcmd = ['123FSET MCV ' num2str(vMax)];
    acmd = ['123FSET ACCEL ' num2str(aMax)];
    
    sendHandCommand(netInfo, uint8(vcmd));
    sendHandCommand(netInfo, uint8(acmd));
end
