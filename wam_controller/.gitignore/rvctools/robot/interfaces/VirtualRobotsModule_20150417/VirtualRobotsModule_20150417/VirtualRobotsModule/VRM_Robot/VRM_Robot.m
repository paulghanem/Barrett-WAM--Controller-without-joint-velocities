%VRM_Robot A class that represents the Virtual Robots visualizer and the associated robots
% 
% A VRM_Robot object holds the information regarding the SerialLink object
% and the Virtual Robots Server that can be used to display the serial link
% robot.
% 
% This class handles the Virtual Robot Module in which the serial chain
% robot scan be visualized
% 
% 
% Methods::
% AvailableRobots()
% LoadRobot()
% DisplayRobot()
% MoveRobot()
% ForwardKinematics()


classdef VRM_Robot < handle
    
	
  
    properties
        jointState;             % Joint angle values associated with the current robot configuration
        initialJointState;      % Initial joint angle values associated with the initial robot configuration for the robot motion
        finalJointState;        % Final joint angle values associated with the final robot configuration for the robot motion
        rtbSerialLinkObject;    % SerialLink object from Robotics Toolbox, that contains information about the robot 
        vrmServer;              %.NET COM Object is used for Visualizing the robot
        jointPositionList;      % List of joint angle values for each time step
        jointVelocityList;      % List of joint velocity values for each timestep
        jointAccelerationList;  % List of joint acceleration values for each timestep
        
    end
	
	properties(Hidden = false)
        defaultJointState;      % The joint state that is read from the XML file
        defaultInitialJointState; 
        defaultFinalJointState;
        defaultCartesianTrajectorySteps = 100;
        defaultTimesteps = 100;          
        
	end
	
	properties(Constant = true, Hidden = true)
        r2d = 180/pi;
        d2r = pi/180;
        
    end
    
    methods
        
        
        % The constructor
        function robot = VRM_Robot()
           % Constructor for the VRM_Robot class. 
           robot.vrmServer = actxserver('IITDelhi.VirtualRobotModuleCOM'); % Link the VRM COM Server to the robot object
           feature('COM_SafeArraySingleDim', 1) % Set the safearray feature as true initially itself
        end        
        
        function AvailableRobots(robot)
            % Method to display the available robots in the RoboAnalyzerVRM
            % install directory
           robot.vrmServer.ShowAvailableRobots() % Displays the availabe robots in the install directory    
           
        end
        
        function LoadRobot(robot,robotName)
            % Method to load an available robot and read its DH parameters. Sets up the Robotics Toolbox's SerialLink object and the associated functionalities.
            robot.rtbSerialLinkObject = createSeriaLinkObject(robot.vrmServer,robotName);
            robot.initialJointState  = robot.rtbSerialLinkObject.theta - [30 30 30 30 30 30]*robot.d2r;
            robot.finalJointState  = robot.rtbSerialLinkObject.theta + [30 30 30 30 30 30]*robot.d2r;
            robot.jointState = robot.rtbSerialLinkObject.theta;
            robot.defaultJointState = robot.jointState;
        end
        
        function DisplayRobot(robot, joint_State)
            if(nargin > 1)
                    AnimateInVRM(robot,joint_State);  
            else                      
                    AnimateInVRM(robot,robot.jointState);                            
            end
        
        end
        
        function MoveRobot(robot, timesteps)
            if (nargin >1)
                    robot.GetJointTrajectory(timesteps);
             else
                    robot.GetJointTrajectory(robot.defaultTimesteps);
            end
               AnimateInVRM(robot,robot.jointPositionList);                            
        end                
        
        function ForwardKinematics(robot, JS_iniitial,JS_final, timesteps)

              if (nargin > 2)
              [robot.jointPositionList, robot.jointVelocityList, robot.jointAccelerationList] = jtraj(JS_iniitial, JS_final, timesteps);               
              else
              [robot.jointPositionList, robot.jointVelocityList, robot.jointAccelerationList] = jtraj(robot.initialJointState, robot.finalJointState, robot.defaultTimesteps);               
              end
                 AnimateInVRM(robot,robot.jointPositionList);   
        end    
        
        function jointAngles =  InverseKinematics(robot,HTM)
             
%              if(nargin >2)
                 jointAngles = InverseKinematics6RWP(robot,HTM)';
%              else
%                  jointAngles = InverseKinematics6RWP(robot, robot.rtbSerialLinkObject.fkine(robot.jointState))';
%              end
        end
        
        function CartesianMotionAbsolute(robot, initialXYZABC, finalXYZABC)
%            htm_initial = transl(initialXYZABC(1:3))*rpy2tr(initialXYZABC,'zyx');
%            htm_final = transl(finalXYZABC(1:3))*rpy2tr(finalXYZABC,'zyx');
           
           steps = robot.defaultCartesianTrajectorySteps;
           changeInConfig = (finalXYZABC - initialXYZABC)/steps;
           
           listOfConfigs  = zeros(6, steps+1);
           listOfAngles =  zeros(6, steps+1);

           listOfConfigs(:,1) = initialXYZABC;
           
           htm_current = transl(listOfConfigs(1:3,1)')*rpy2tr(listOfConfigs(4:6,1)','zyx');
           ikin_current = robot.InverseKinematics(htm_current);
           
%          To select the best starting configuration
           for i = 1: size(ikin_current,2)                
                penalty(i) = norm(robot.defaultJointState(1:3)' - ikin_current(1:3,i));
           end

           [minval indexOfSol ] = min(penalty);
           ikin_current(:,indexOfSol);
           listOfAngles(:,1) =  ikin_current(:,indexOfSol);


           % To choose the solution for the next step
           for i  = 2:steps+1
    
                listOfConfigs(:,i) = listOfConfigs(:,i-1) +  changeInConfig;
    
                htm_next = transl(listOfConfigs(1:3,i)')*rpy2tr(listOfConfigs(4:6,i)','zyx');
                ikin_next= robot.InverseKinematics(htm_next);
    
                penalty = zeros(length(size(ikin_next,2)));
                
                for j = 1:size(ikin_next,2)                    
                    penalty(j) = norm(listOfAngles(1:6,i-1) - ikin_next(1:6,j));
                end
        
                [minval, indexOfSol ] = min(penalty);
                listOfAngles(:,i) =  ikin_next(:,indexOfSol); 
           end
           robot.jointState = listOfAngles(:,i)';
           
           
            AnimateInVRM(robot,listOfAngles');
           
           
           
        end
        
        
        function CartesianMotionRelative(robot,changeInConfig) %showAnimation)
                          
                initialTMatrix = robot.rtbSerialLinkObject.fkine(robot.jointState);
                initialConfig(1:3,1) = initialTMatrix(1:3,4);
                initialConfig(6:-1:4,1) = tr2rpy(initialTMatrix,'zyx'); %returns as C,B,A. Hence the discrepancy in indices
                
                finalConfig(1:3,1) = initialConfig(1:3,1) + changeInConfig(1:3,1);
                finalConfig(4:6,1) = initialConfig(4:6,1) + changeInConfig(4:6)*pi/180;
                
                changeInTwist = (finalConfig - initialConfig)/robot.defaultCartesianTrajectorySteps;
                
                jv(1,:) = robot.jointState;
                temp = eye(size(changeInTwist,1));
                
                for i = 2:robot.defaultCartesianTrajectorySteps                    
                       rpy = tr2rpy(robot.rtbSerialLinkObject.fkine(jv(i-1,:)),'zyx');  % rpy in degrees
                       A = rpy(3);  B = rpy(2) ; C = rpy(1);
                       temp(4:6,4:6) = [1      0         sin(B) ;...
                                        0 cos(A) -sin(A)*cos(B) ;...
                                        0 sin(A)  cos(A)*cos(B)];     

                       del_jv = pinv(robot.rtbSerialLinkObject.jacob0(jv(i-1,:)))*temp*changeInTwist;
                       jv(i,:) = jv(i-1,:) + del_jv';
                end
                
                robot.jointState = jv(i,:);
                
%                 if (showAnimation == true)
                AnimateInVRM(robot,jv);   
%                 end
              
        end
       
        
        
%       The class destructor
        function delete(robot)
             robot.vrmServer.DisposeRobot; 
        end
                        
        % get set methods
        
        function set.jointState(robot,JState)
            robot.jointState = JState;
        end
        
        function set.initialJointState(robot,JState)
%               Sets the initial joint state of the robot to the required
%               value
              robot.initialJointState = JState;
         end
    
        function set.finalJointState(robot,JState)
%               Sets the final joint state of the robot to the required
%               value
              robot.finalJointState = JState;
        end
                
    end % methods
    
    methods (Hidden = true)
        
        function GetJointTrajectory(robot,timesteps)
            if (nargin > 1)
            [robot.jointPositionList robot.jointVelocityList robot.jointAccelerationList] = jtraj(robot.initialJointState, robot.finalJointState, timesteps);               
            else
            [robot.jointPositionList robot.jointVelocityList robot.jointAccelerationList] = jtraj(robot.initialJointState, robot.finalJointState, robot.defaultTimesteps);               
            end         
        end
        
        function TRF = GetTransformationMatrix(robot,jointState)
             if(nargin>1)
                 TRF = robot.rtbSerialLinkObject.fkine(jointState);
             else
                 TRF = robot.rtbSerialLinkObject.fkine(robot.jointState);
             end
        end
        
    end %methods (Hidden = true)
    
end % class

