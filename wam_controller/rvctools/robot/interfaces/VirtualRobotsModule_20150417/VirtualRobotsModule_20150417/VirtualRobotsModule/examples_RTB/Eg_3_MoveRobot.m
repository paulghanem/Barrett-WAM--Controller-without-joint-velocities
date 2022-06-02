%% Example 3: Create a robot object and display the robot in a required joint state
clear
clc

% Create the Virtual Robots server object
myRobot =  VRM_Robot()

% Load a robot for kinematic analysis from the list of available robots
myRobot.LoadRobot('kukakr5');
% myRobot.DisplayRobot()
disp(' Moving the Robot from the initial to final configuration')

% Display the robot in the  new joint configuration
myRobot.MoveRobot


% myRobot.MoveRobot(300); % Pass the required time steps as the argument




