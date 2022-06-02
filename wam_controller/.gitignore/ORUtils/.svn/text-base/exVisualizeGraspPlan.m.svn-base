function [ objectid planStruct object_T plotIDs] = exVisualizeGraspPlan( grasp_plan_filename, tformInfo, ttProjectFile,waypointIndex,experiment,callbackData )
%EXVISUALIZEGRASPPLAN Visualizes the grasp plan using input data from the tracking system
%   The function parse the grasp plan file first
%   Tries to find the object using the tracking system
%   Load in the object specified in the grasp plan into openRAVE
%   Places the object in the scene at the located position
%   Plot all grasps in the plan text file
%   If the robotid is given, will display robot arm at all pregrasp positions, one at a time

    % Clear pregrasp frames -> plotIDs are outputted to clear frames run orEnvClose(plotIDs)
    
    global arm_ready;

    % Parse the grasp plan file first
    planStruct = loadGraspPlan( grasp_plan_filename );

    
    % Do tracking system initilization if we are not in an experiment
    if nargin < 5
        % if something screws up, shutdown the system gracefully
        c = onCleanup(@()TTSHUTDOWN());

        % A few constants
        FRAME_BUFF_SIZE = 1000;
        MARKER_BUFF_SIZE = 10;
        
        % Try to find the object in the tracking stystem
        % initialize tracking system
        fprintf('Initializing Tracking System...\n');
        [trackData] = initTrackingSystem(ttProjectFile, FRAME_BUFF_SIZE, MARKER_BUFF_SIZE);

        % Put trackData in experiment so that we can use the already written functions
        experiment.trackData = trackData;

        % get tracking frame from the camera system
        [trackDataRcvd] = getTrackingFrame(experiment,tformInfo.Tcamwam,1);
        
        % locate the tray in the list of trackables
        experiment.objId = getTrackableIdByName(experiment.trackData.trackables, 'Tray');
        if(experiment.objId == 0)
            error('There is no trackable named "Tray"');
        end
    else
       trackDataRcvd = callbackData.trackDataRcvd; 
    end
    
    TWAMTray = trackDataRcvd.trackables(experiment.objId).Tobjwam

%     TWAMTray = [  -1.0000         0         0   0.70
%                      0   -1.0000         0   -0.2500
%                      0         0    1.0000   -0.2700
%                      0         0         0    1.0000];
     TTrayObj = getTTrayObj(planStruct.objectId);
     object_T = TWAMTray * TTrayObj

    % If the body already exists, just move it.
    objectid = 0;
    bodies = cell2mat(orEnvGetBodies());
    for i = 1:length(bodies)
        if(strcmp(bodies(i).name, 'object'))
            objectid = i;
        end
    end
    
    if(objectid == 0)
        % Load the object into openRAVE
        objectid = orEnvCreateKinBody('object', ['data/harris' planStruct.objectId '.kinbody.xml']);
    end
    
    % Convert the 3*4 transformation matrix (Top three rows of the homogeneous
    % transformation matrix) into a column vector, as required by the function
    orBodySetTransform(objectid, reshape(object_T(1:3, :), [1 12]));

    % Plot the grasp plans at pre-grasp configurations
    graspPs = zeros(planStruct.numberOfPlans, 3);
    approachVs = zeros(planStruct.numberOfPlans, 3);
    openingVs = zeros(planStruct.numberOfPlans, 3);

    validPlansIndex = zeros(planStruct.numberOfPlans,1);
    
    for i = 1:planStruct.numberOfPlans
        %pregraspT = object_T * planStruct.plans(i).waypoints(waypointIndex).Htransform; % If plans are in object frame
        pregraspT = planStruct.plans(i).waypoints(waypointIndex).Htransform; % If plans are in world frame
       % finalgraspT = planStruct.plans(i).waypoints(4).Htransform;

        [valid q] = wam7ik_w_joint_limits(pregraspT,arm_ready');

        if valid
            validPlansIndex(i) = 1;
        end
        
        graspPs(i, :) = pregraspT(1:3, 4)';
        approachVs(i, :) = pregraspT(1:3, 3)';
        openingVs(i, :) = pregraspT(1:3, 2)';
    end
    
    
%     for i = 1:5
%         validPlansIndex(i) = 1;
%         pregraspT = object_T * planStruct.plans(1).waypoints(i).Htransform;
%         graspPs(i, :) = pregraspT(1:3, 4)';
%         approachVs(i, :) = pregraspT(1:3, 3)';
%         openingVs(i, :) = pregraspT(1:3, 2)';
%     
%     end
    
    
    fprintf('Displaying valid plans\n');
    if(sum(validPlansIndex) > 0)
        plotIDs = plotGraspT(graspPs, approachVs, openingVs, validPlansIndex);
    else
        warning('No plans can be reached at the current object position');
        plotIDs = [];
    end
    
    fprintf('Done!!\n');
    
end

% This function plot a grasp pose into the scene of openRAVE, the wrist
% position will be plotted as a sphere, the approach vector of the gripper
% will be plotted as a longer line segment starting from the sphere, the
% openning direction of the gripper will be plotted as a shorter segment
% the openning direction vector and the approach vector forms the gripper
% surface
function plotIDs = plotGraspT(p, av, ov, validPlansIndex)

%p = T(1:3, 4)';
%approach_vector = T(1:3, 3)';
%openning_vector = T(1:3, 2)';

sphere_radius = 0.005;
long_segment =  0.08;
short_segment = 0.04;
line_width = 1;
fontsize = 16;
plotIDs = zeros(1,2+2*sum(validPlansIndex));

strs = [];
for i = 1:size(p, 1)
    if i < 10
        strs = [strs; '0' num2str(i)];
    else
       strs = [strs; num2str(i)]; 
    end 
end

Dstrs = strs(validPlansIndex == 1,:);
Dp = p(validPlansIndex == 1,:);
Dav = av(validPlansIndex == 1,:);
Dov = ov(validPlansIndex == 1,:);

% Plot the wrist position
plotIDs(1) = orEnvPlot(Dp, 'color', [1, 1, 0], 'size', sphere_radius, 'sphere');

strs = [];
for i = 1:size(p, 1)
    if i < 10
        strs = [strs; '0' num2str(i)];
    else
       strs = [strs; num2str(i)]; 
    end 
end

plotIDs(2) = orEnvDisplayText(Dp, Dstrs, 'size', fontsize);

% Plot the approach vector
endpoint1 = Dp + long_segment * Dav;
endpoint2 = Dp + short_segment * Dov;
for i = 1:size(Dp, 1)
    plotIDs(i*2+1) = orEnvPlot([Dp(i, :); endpoint1(i, :)], 'color', [0, 0, 1], 'size', line_width, 'line');
    plotIDs(i*2+2) = orEnvPlot([Dp(i, :); endpoint2(i, :)], 'color', [0, 1, 0]    , 'size', line_width, 'line');
end

end

function TTSHUTDOWN()
    try
        if(libisloaded('NPTrackingTools'))
            calllib('NPTrackingTools', 'TT_Shutdown');
        end
    catch
    end
end

