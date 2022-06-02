% from Craig, pp. 55-56
function R = getQuaternion2Rot(qtr)
    R = [1-2*qtr(2)^2-2*qtr(3)^2, 2*(qtr(1)*qtr(2)-qtr(3)*qtr(4)), 2*(qtr(1)*qtr(3)+qtr(2)*qtr(4));
        2*(qtr(1)*qtr(2)+qtr(3)*qtr(4)), 1-2*qtr(1)^2-2*qtr(3)^2, 2*(qtr(2)*qtr(3)-qtr(1)*qtr(4))
        2*(qtr(1)*qtr(3)-qtr(2)*qtr(4)), 2*(qtr(2)*qtr(3)+qtr(1)*qtr(4)), 1-2*qtr(1)^2-2*qtr(2)^2];
end
