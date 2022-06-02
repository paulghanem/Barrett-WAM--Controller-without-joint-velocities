%% Example 4: Create a robot object and display the foreard kinematics motion
clear
clc

% Create the Virtual Robots server object
myRobot =  VRM_Robot()

% Load a robot for kinematic analysis from the list of available robots
myRobot.LoadRobot('kukakr5');

% Define the initial and final configuration for the forward kinematics simulation
initialConfig = [ 120 60 30 60 0  130]*pi/180
finalConfig   = [-120 90 -45 90 0 -130]*pi/180
timesteps = 150;

disp('Forward Kinematics simulation for motion from initialConfig to finalConfig');

% Start the motion in the Virtual Robots Module
myRobot.ForwardKinematics(initialConfig, finalConfig ,timesteps)





    



