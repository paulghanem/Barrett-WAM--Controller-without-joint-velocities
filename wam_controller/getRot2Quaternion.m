% from Craig, pp. 55-56
function qtr = getRot2Quaternion(R)
    qtr = nan(1,4);
    
    % qtr = [qx qy qz qw], same as eps = [eps1 eps2 eps3 eps4] in Craig
    qtr(4) = sqrt(1+R(1,1)+R(2,2)+R(3,3))/2;
    qtr(1) = (R(3,2)-R(2,3))/(4*qtr(4));
    qtr(2) = (R(1,3)-R(3,1))/(4*qtr(4));
    qtr(3) = (R(2,1)-R(1,2))/(4*qtr(4));
end
