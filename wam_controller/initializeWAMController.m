% This file initialize the WAM Controller by loading the necssary variables and 
% adding folder paths to the utility functions that will be used

% Add necessary folders so that commands are easily accessible
addpath C:\Users\Robotics\Documents\MATLAB\wam_controller\WAM7DOF\
addpath C:\Users\Robotics\Documents\MATLAB\wam_controller\WAM7DOF\ArmCommands\
addpath C:\Users\Robotics\Documents\MATLAB\wam_controller\WAM7DOF\WSGCommands\
addpath C:\Users\Robotics\Documents\MATLAB\wam_controller\WAM7DOF\ExperimentCallbacks\
addpath C:\Users\Robotics\Documents\MATLAB\wam_controller\WAM7DOF\GraspPlans\
addpath C:\Users\Robotics\Documents\MATLAB\wam_controller\WAM7DOF\Utilities\ 
addpath C:\Users\Robotics\Documents\MATLAB\wam_controller\WAM7DOF\ORUtils\
addpath C:\Users\Robotics\Documents\MATLAB\wam_controller\WAM7DOF\TrajectoryGeneration\
addpath C:\Users\Robotics\Documents\MATLAB\wam_controller\WAM7DOF\TrajectoryGeneration\invKinSearch\
addpath C:\Users\Robotics\Documents\MATLAB\wam_controller\WAM7DOF\TrajectoryGeneration\jointSpaceTraj\
addpath 'C:\Program Files (x86)\OpenRAVE\share\openrave-0.6\matlab\'
addpath 'C:\Program Files (x86)\OpenRAVE\bin\'

addpath 'C:\Users\robotics\Documents\MATLAB\wam_controller\matlab_graspit_interface\matlab\interface\'
addpath 'C:\Users\robotics\Documents\MATLAB\wam_controller\matlab_graspit_interface\matlab\src\'
addpath 'C:\Users\robotics\Documents\MATLAB\wam_controller\matlab_graspit_interface\matlab\src\scenarios\'
addpath 'C:\Users\robotics\Documents\MATLAB\wam_controller\matlab_graspit_interface\matlab\src\grasp_planning_for_scenario\'
addpath 'C:\Users\robotics\Documents\MATLAB\wam_controller\matlab_graspit_interface\matlab\src\grasp_planning_for_scenario\handle_occupancy_grid'
addpath 'C:\Users\robotics\Documents\MATLAB\wam_controller\matlab_graspit_interface\matlab\src\mexs\'
addpath 'C:\Users\robotics\Documents\MATLAB\wam_controller\matlab_graspit_interface\matlab\src\utils\'
addpath 'C:\Users\robotics\Documents\MATLAB\wam_controller\matlab_graspit_interface\matlab\src\execution\'
addpath 'C:\Users\robotics\Documents\MATLAB\wam_controller\matlab_graspit_interface\matlab\depend\'
addpath 'C:\Users\robotics\Documents\MATLAB\wam_controller\matlab_graspit_interface\matlab\depend\robotics_toolbox\'
addpath 'C:\Users\robotics\Documents\MATLAB\wam_controller\matlab_graspit_interface\matlab\depend\robotics_toolbox\rvctools\'
addpath 'C:\Users\robotics\Documents\MATLAB\wam_controller\matlab_graspit_interface\matlab\depend\tcpip_toolbox\'
addpath 'C:\Users\robotics\Documents\MATLAB\wam_controller\matlab_graspit_interface\matlab\depend\tcpip_toolbox\tcp_udp_ip\'
% addpath C:\Users\Robotics\Documents\MATLAB\wam_controller\WAM7DOF\BarrettHandCommands\
% addpath C:\Users\Robotics\Documents\MATLAB\wam_controller\WAM7DOF\DatabaseCommands\

startup_rvc

% Load workspace variables needed to run the controller
load wam7_init.mat
load tformInfo20120520.mat
load CALIBRATION3.mat

wamInfo = initWamInfo();

% Open xPC Target Explorer in case user want to issue their own commands
xpcexplr

