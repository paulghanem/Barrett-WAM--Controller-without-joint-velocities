% from Craig, pp. 45-49
function R = getEuler2Rot(ypr)
    % z-y-x notation (yaw(z), then pitch(y), then roll(x))
    R = [cos(ypr(2))*cos(ypr(1)), -cos(ypr(3))*sin(ypr(1))+sin(ypr(3))*sin(ypr(2))*cos(ypr(1)),  sin(ypr(3))*sin(ypr(1))+cos(ypr(3))*sin(ypr(2))*cos(ypr(1));
         cos(ypr(2))*sin(ypr(1)),  cos(ypr(3))*cos(ypr(1))+sin(ypr(3))*sin(ypr(2))*sin(ypr(1)), -sin(ypr(3))*cos(ypr(1))+cos(ypr(3))*sin(ypr(2))*sin(ypr(1));
        -sin(ypr(2)), sin(ypr(3))*cos(ypr(2)), cos(ypr(3))*cos(ypr(2))];
end
