function [ robotid, objectid, validPlansStruct ] = visualizeGraspPlan( grasp_plan_filename, robot_base_T, table_T, object_T, robot_joint_angles)
%VISUALIZEGRASPPLAN This function visualize the grasp plans in the grasp
%plan file in OpenRAVE to help decide which grasps are reachable from
%current robot configuration
%   The function parse the grasp plan file first
%   Load in the robot into openRAVE
%   Load in table into openRAVE
%   Load in the object specified in the grasp plan into openRAVE
%   Plot all grasps in the plan text file

global arm_ready;

% Parse the grasp plan file first
planStruct = loadGraspPlan( grasp_plan_filename );
fprintf('Total Plans: %d\n', planStruct.numberOfPlans);

fprintf('Initializing openRAVE scene\n');
% Load barrett wam into openRAVE
robotid = orEnvCreateRobot('barrettwam', 'robots/barrettwam.robot.xml');
% Convert the 3*4 transformation matrix (Top three rows of the homogeneous
% transformation matrix) into a column vector, as required by the function
orBodySetTransform(robotid, reshape(robot_base_T(1:3, :), [1 12]));
% Set the robot joint angles
orRobotSetDOFValues(robotid, robot_joint_angles, 0:6)

% Load table into openRAVE
tableid = orEnvCreateKinBody('table', 'data/table.kinbody.xml');
% Convert the 3*4 transformation matrix (Top three rows of the homogeneous
% transformation matrix) into a column vector, as required by the function
orBodySetTransform(tableid, reshape(table_T(1:3, :), [1 12]));

% Load the object into openRAVE
objectid = orEnvCreateKinBody('object', ['data/harris' planStruct.objectId '.kinbody.xml']);
% Convert the 3*4 transformation matrix (Top three rows of the homogeneous
% transformation matrix) into a column vector, as required by the function
orBodySetTransform(objectid, reshape(object_T(1:3, :), [1 12]));

fprintf('Finding valid plans\n');
% Plot the grasp plans at pre-grasp configurations
graspPs = zeros(planStruct.numberOfPlans, 3);
approachVs = zeros(planStruct.numberOfPlans, 3);
openingVs = zeros(planStruct.numberOfPlans, 3);

validPlans = 0;
validPlansStruct = struct();

for i = 1:planStruct.numberOfPlans
    pregraspT = object_T * planStruct.plans(i).waypoints(1).Htransform;
   % finalgraspT = planStruct.plans(i).waypoints(4).Htransform;
   
    [valid q] = wam7ik_w_joint_limits(pregraspT,arm_ready');
    
    if(valid)
        validPlans = validPlans+1;
        validPlansStruct.plans(validPlans) = planStruct.plans(i);
        graspPs(validPlans, :) = pregraspT(1:3, 4)';
        approachVs(validPlans, :) = pregraspT(1:3, 3)';
        openingVs(validPlans, :) = pregraspT(1:3, 2)';
    end
end


validPlansStruct.numberOfPlans = validPlans;

fprintf('Displaying valid plans\n');
plotGraspT(graspPs, approachVs, openingVs);

fprintf('Done!!');

end

% This function plot a grasp pose into the scene of openRAVE, the wrist
% position will be plotted as a sphere, the approach vector of the gripper
% will be plotted as a longer line segment starting from the sphere, the
% openning direction of the gripper will be plotted as a shorter segment
% the openning direction vector and the approach vector forms the gripper
% surface
function plotGraspT(p, av, ov)

%p = T(1:3, 4)';
%approach_vector = T(1:3, 3)';
%openning_vector = T(1:3, 2)';

sphere_radius = 0.005;
long_segment =  0.08;
short_segment = 0.04;
line_width = 1;
fontsize = 16;

% Plot the wrist position
orEnvPlot(p, 'color', [1, 1, 0], 'size', sphere_radius, 'sphere');

strs = [];
for i = 1:size(p, 1)
    if i < 10
        strs = [strs; '0' num2str(i)];
    else
       strs = [strs; num2str(i)]; 
    end 
end
orEnvDisplayText(p, strs, 'size', fontsize);

% Plot the approach vector
endpoint1 = p + long_segment * av;
endpoint2 = p + short_segment * ov;
for i = 1:size(p, 1)
    orEnvPlot([p(i, :); endpoint1(i, :)], 'color', [0, 0, 1], 'size', line_width, 'line');
    orEnvPlot([p(i, :); endpoint2(i, :)], 'color', [0, 1, 0]    , 'size', line_width, 'line');
end

end

