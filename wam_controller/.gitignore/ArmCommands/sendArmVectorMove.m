function sendArmVectorMove( netInfo, wamInfo,q, v )
%SENDARMVECTORMOVE Sends a Cartesian space move by a certain vector in the world frame
%   Detailed explanation goes here
    
    T = getTransformN2Base(7,wamInfo,q);
    
    T(1:3,4) = T(1:3,4) + v;
    
    sendArmCspaceMove(netInfo,T);

end

