% these axis angle functions work for 0 degrees using a hack
% but do not work for 180 degrees at all
% we need to implement quaternions+SLERP here and in the simulink model
function [w] = getRot2AxisAngle(R)
    theta = acos((R(1,1)+R(2,2)+R(3,3)-1)/2);
    if(abs(pi-theta) < 1e-3)
%         180 needs to be fixed, should probably just use quaternions
%         R = R*getEuler2Rot([1e-4 1e-4 1e-4]);
%         theta = acos((R(1,1)+R(2,2)+R(3,3)-1)/2);
        error('180 deg rotation singularity');
    end
    
    if(abs(theta) < 1e-3)
        % doesnt matter which axis we pick
        K = [0 0 1]';
        theta = 0;
    else
        K = [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)]./(2*sin(theta));
    end

    w = K*theta;
end
