function AnimateInVRM(robot,JLIST)
%           ANIMATEINVRM Animates the robot motion in Virtual Robots Addin for MATLAB
% 
%           ANIMATEINVRM(robot,JLIST) opens the Virtual Robots visualizer
%           and displays the serial robot corerspoding to the object robot. 
%           The joint angles are moved as per the values given in JLIST,
%           which is a nx6 matrix, for n timesteps
            
            
            steps = size(JLIST,1);
            JLIST = JLIST*180/pi;
            
            robot.vrmServer.DisplayRobot();    
            
            jointArray = zeros(size(JLIST,2),1);
            for i = 1:steps
                for j = 1:size(JLIST,2)
                    jointArray(j) = JLIST(i,j); %Set values of each joint angle in MATLAB, angles in degree
                end
               robot.vrmServer.UpdateRobot(jointArray) % Update the joint angles of Virtual Kuka by sending MATLAB Array
               pause(0.01) % Acts as a time interval between two frames of animation.,    
            end

end