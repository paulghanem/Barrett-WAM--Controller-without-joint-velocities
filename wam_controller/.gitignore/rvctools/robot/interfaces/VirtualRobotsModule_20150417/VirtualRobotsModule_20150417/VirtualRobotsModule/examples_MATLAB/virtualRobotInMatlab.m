%Connect to VirtualKUKA Application using COM and ActiveX
virtualRobot = actxserver('IITDelhi.VirtualRobotModuleCOM')

%the array to be sent to C# COM object should be SafeArray
feature('COM_SafeArraySingleDim', 1) 

%virtualRobot.ShowAvailableRobots %shows the names of the avaialable robots

%starts virtual robot in a new form
virtualRobot.LoadRobot('kukakr5') 

virtualRobot.DisplayRobot %form is shown., now., you can change the joint angles by sending array

jointArray = zeros(6,1);

for i = 1:90
   jointArray(1) = i; %Set values of each joint angle in MATLAB, angles in degree
   jointArray(2) = 90-i*0.5;
   jointArray(3) = -i*0.25;
   jointArray(4) = i;
   jointArray(5) = 90-i;
   jointArray(6) = i;
   virtualRobot.UpdateRobot(jointArray) % Update the joint angles of Virtual Kuka by sending MATLAB Array
   
   pause(0.05) % Acts as a time interval between two frames of animation.,    
end
