table_T = [0 1 0 0.58; -1 0 0 -.04; 0 0 1 -.346; 0 0 0 1];
object_T = [1 0 0 .5; 0 1 0 .2; 0 0 1 -.2; 0 0 0 1];
robot_joint_angles = arm_home_norest;
robot_base_T = [1 0 0 -.22; 0 1 0 -.14; 0 0 1 -.346; 0 0 0 1];

[robotid objectid planstruct] = visualizeGraspPlan('planresult_Perception_Scenario_1.txt', robot_base_T, table_T, object_T, robot_joint_angles);

fprintf('Valid Plans: %d\n', planstruct.numberOfPlans);

for i = 1:planstruct.numberOfPlans   
    exDisplayPreGrasp(robotid, objectid, planstruct, i, object_T);
    pause(.25);
end

