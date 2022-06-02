%% Example 2: Create a robot object and display the robot in a required joint state
clear
clc

% Create the Virtual Robots server object
myRobot =  VRM_Robot()

% Load a robot for kinematic analysis from the list of available robots
myRobot.LoadRobot('kukakr5');

% Display the robot
myRobot.DisplayRobot();  % Shows the robot int he Virtual Robots mOdule in the default joint configuration

pause(3); % Hold the view for some time

% Define a new joint configuration
newRobotConifg = [120 60 45 0 70 120]*pi/180;

% Display the robot in the  new joint configuration
myRobot.DisplayRobot(newRobotConifg)

disp(' Displaying the robot in the new configuration')