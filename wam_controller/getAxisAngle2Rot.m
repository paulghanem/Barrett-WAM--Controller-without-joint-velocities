% these axis angle functions work for 0 degrees using a hack
% but do not work for 180 degrees at all
% we need to implement quaternions+SLERP here and in the simulink model
function [ R ] = getAxisAngle2Rot(w)
    thK = sqrt(sum(w.^2));
    if(abs(thK) < 1e-4 || abs(pi-thK) < 1e-4)
        thK = 0;
        K = [0 0 1]';
    else
        K = w./thK;
    end

    Kx = K(1);
    Ky = K(2);
    Kz = K(3);
    
    ct = cos(thK);
    st = sin(thK);
    vt = 1-cos(thK);
    
    R = [Kx*Kx*vt+ct, Kx*Ky*vt-Kz*st, Kx*Kz*vt+Ky*st;
         Kx*Ky*vt+Kz*st, Ky*Ky*vt+ct, Ky*Kz*vt-Kx*st;
         Kx*Kz*vt-Ky*st, Ky*Kz*vt+Kx*st, Kz*Kz*vt+ct];
end
