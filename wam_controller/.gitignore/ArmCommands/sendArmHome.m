function sendArmHome(netInfo,wamInfo,qd,qdd)
    % go slightly above home position to avoid twist against rubber stop
    q = wamInfo.arm.home_norest;
    
    % set defaults if either acceleration or velocity is not provided
    if(nargin < 4)
        qd = 0.5*ones(1,7);
        qdd = 0.5*ones(1,7);
    end
    
    sendArmJspaceMove(netInfo,q,qd,qdd);
end
