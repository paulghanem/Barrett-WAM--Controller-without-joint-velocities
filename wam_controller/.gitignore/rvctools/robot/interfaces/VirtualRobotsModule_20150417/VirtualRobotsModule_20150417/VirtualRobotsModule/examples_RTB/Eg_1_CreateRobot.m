%% Example 1: Create a robot object using the Virtual Robots Module and Robotics Toolbox

clear
clc

% Create the Virtual Robots server object
myRobot =  VRM_Robot()

% See the list of available robots
myRobot.AvailableRobots();

% Load a robot for kinematic analysis from the list of available robots
myRobot.LoadRobot('kukakr5');

% Access the functionalities of the SerialLink classs from Robotics Toolbox
myRobot.rtbSerialLinkObject()