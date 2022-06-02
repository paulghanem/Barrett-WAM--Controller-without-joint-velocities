% this function is old!! doesnt work
function w = getToolConfig(wamInfo,q)
    q(5) = 0;

    T = getTransformN2Base(5,wamInfo,q);

    % no tool roll so exp(qn/pi) = exp(0) = 1
    % so last 3 rows are simply the z-axis of the tool
    % oops, this only defines yaw and pitch of the tool, but not roll
    % (Fundamentals of Robotics, Schilling pp. 87)
%     w = [ T(1:3,4); T(1:3,3) ];

    R = T(1:3,1:3);
    
    % using axis-angle, K represented in the base frame
    % Craig pp. 51-53
    % allowing K and theta to be *anything* is pretty unintuitive when trying
    % to form a trajectory... this needs more work (or 7 DOF)
    theta = acos((R(1,1)+R(2,2)+R(3,3)-1)/2);
    K = (1/(2*sin(theta)))*[R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
    
    K,theta,K*exp(theta/pi),sum(K.^2)
    
    w = [T(1:3,4); K*exp(theta/pi)];
end
