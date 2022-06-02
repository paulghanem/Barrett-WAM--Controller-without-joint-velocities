function SerialLinkRobot = createSeriaLinkObject( vrmServerObject,robotName)
% createSeriaLinkObject Creates the SerialLinkObject for the robot using
% the SerialLink() methods of Robotics Toolbox
% 
% SerialLinkRobot = createSeriaLinkObject(vrmServerObject, robotName)
%                   Takes as input the vrmServer object and the name of the robot
%                   availabe in the install directory. The robot parameters
%                   required for creating the SerialLink() object are
%                   obtained form the mehtods of the VRM Server


% Load the robot.xml using the VRM Server and construct the form
vrmServerObject.LoadRobot(robotName);

% Get the no of joints in the robot
noOfJoints = vrmServerObject.GetNoOfJoints;

% Read teh DH parameter from robot.xml using the VRM Server and create the
% links

% Create the individual links using the methods from Robotivsd
%     LINK [theta d a alpha]celar
for i =1:noOfJoints
    L(i,:)  = Link( [   vrmServerObject.GetJointAngle(int2str(i))*pi/180,...                                         
                        vrmServerObject.GetJointOffset(int2str(i))/1000,...
                        vrmServerObject.GetLinkLength(int2str(i))/1000,...   
                        vrmServerObject.GetTwistAngle(int2str(i))*pi/180]  );
    L(i).qlim = [vrmServerObject.GetJointVariableLimitMin(int2str(i)) , vrmServerObject.GetJointVariableLimitMax(int2str(i)) ];
end

tempRobot = SerialLink(L);
tempRobot.name = robotName;

SerialLinkRobot = tempRobot;
end

