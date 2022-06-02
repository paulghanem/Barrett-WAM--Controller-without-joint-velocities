function [ valid, q ] = wam7_straightline_traj_gen( start_joint_configuration, endpointT)
%the path between the initial Position and waypoint1 will just try to avoid
%joint limits and the table, the
d7 = 0.154;% meters
endpointT = endpointT/transl(0, 0, d7);

valid = true;
resStraight = 20;

q3res = 1;
q = [];
while(q3res < 128)
    try
        q_straight = trajTest(start_joint_configuration, endpointT, resStraight, q3res);
        q = q_straight;
        return;
    catch e
        disp(e.message);
        q3res = q3res*2;
    end
end
if q3res >= 128
    disp('Error finding straight path');
    valid = false;
    return;
end

end

