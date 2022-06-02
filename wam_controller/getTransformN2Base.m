function T0 = getTransformN2Base(N,wamInfo,q)
    % compute forward kinematics
    % T0 is T(N to base)
    T0 = eye(4);
    for i=1:N
        T = getTransformDH(wamInfo.arm.dh.a(i), wamInfo.arm.dh.d(i), wamInfo.arm.dh.alpha(i), q(i));
        T0 = T0*T;
    end
end
