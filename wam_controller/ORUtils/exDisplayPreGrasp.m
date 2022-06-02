function exDisplayPreGrasp( robotid, planStruct, i, object_T,objectid )
%EXDISPLAYPREGRASP Displays the WAM ik solution to the ith pregrasp
%   Detailed explanation goes here

    global arm_ready;

    pregraspT = object_T * planStruct.plans(i).waypoints(1).Htransform;
    % finalgraspT = planStruct.plans(i).waypoints(4).Htransform;

    [valid q] = wam7ik_w_joint_limits(pregraspT,arm_ready');
    
    if valid
       orRobotSetDOFValues(robotid,q,0:6); 
    else
        error('There was not a valid IK solution for this pregrasp.')
    end
end

