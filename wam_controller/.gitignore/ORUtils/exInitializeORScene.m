function [ robotid ] = exInitializeORScene( )
%EXINITIALIZEORSCENE Initializes the OpenRave scene that will be used to
%visualize pregrasp poses.
%   This function
%      adds the WAM to the scene so that the WAM's origin is at the scenes origin
%      add the table to the scene
%      put the WAM in arm_home_norest
%   
%   Returns the robot_id in the scene.

global arm_home_norest

table_T = [0 1 0 0.58; -1 0 0 -.04; 0 0 1 -.3; 0 0 0 1];
% Offsets taken from base to transformation of Xbase to X0 in the WAM manual
robot_base_T = [1 0 0 -.22; 0 1 0 -.14; 0 0 1 -.346; 0 0 0 1];

% Load barrett wam into openRAVE
robotid = orEnvCreateRobot('BarrettWAM', 'robots/barrettschunkhand.robot.xml');
% Convert the 3*4 transformation matrix (Top three rows of the homogeneous
% transformation matrix) into a column vector, as required by the function
orBodySetTransform(robotid, reshape(robot_base_T(1:3, :), [1 12]));
% Set the robot joint angles
orRobotSetDOFValues(robotid, [arm_home_norest(:)' 0 0], 0:8)

% Load table into openRAVE
tableid = orEnvCreateKinBody('table', 'data/table.kinbody.xml');

% Convert the 3*4 transformation matrix (Top three rows of the homogeneous
% transformation matrix) into a column vector, as required by the function
orBodySetTransform(tableid, reshape(table_T(1:3, :), [1 12]));

end

