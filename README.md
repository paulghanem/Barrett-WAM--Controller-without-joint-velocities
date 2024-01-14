# Barrett WAM control without joint velocities
This is the companion code for the robot control method reported in the paper A multivariable stochastic tracking controller for robot manipulators without joint velocities by Samer Saab and Paul Ghanem, TAC 2017. The paper can be found here (https://ieeexplore.ieee.org/abstract/document/8100950). The code allows the users to reproduce the results reported in the benchmark. Please cite the above paper when reporting, reproducing or extending the results.
# Purpose of the project
This software is a research prototype, solely developed for and published as part of the publication cited above. It will neither be maintained nor monitored in any way.
# Requirements, how to build, test, install, use, etc.
The code depends on MATLAB.
# Reproducing results
The experiments reported in the publication can be run by executing 

Barrett_WAM/wam_controller/Controllers/Backstepping
/stochastic_backstepping.m and 
Barrett_WAM/wam_controller/Controllers/Backstepping/Stochastic_Backstepping_4dof.m
