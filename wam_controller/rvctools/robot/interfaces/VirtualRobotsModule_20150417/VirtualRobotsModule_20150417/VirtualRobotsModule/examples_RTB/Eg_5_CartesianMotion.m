%% Example 4: Create a robot object and perform motion in the Cartesian space
clear
clc

% Create the Virtual Robots server object
myRobot =  VRM_Robot()

% Load a robot for kinematic analysis from the list of available robots
myRobot.LoadRobot('kukakr5');

% Define the initial and final configuration for the forward kinematics simulation


% The robot configuration is represented using the End Effector position - (X,Y,Z) and the RPY angles of the end effector - (A,B,C)
myRobot.CartesianMotionRelative([-0.3  0.3  0.1  30 50 70]')  % The elements in the vector denote deltaX, deltaY, deltaZ, deltaA, deltaB, and deltaC
% myRobot.CartesianMotionRelative([-0.1 0 0 0 0 0]')
% myRobot.CartesianMotionRelative([0   0.2 0 0 0 0]')
% myRobot.CartesianMotionRelative([0   0 -0.1 0 0 0]')
% myRobot.CartesianMotionRelative([0   0 0  30 0 0]')