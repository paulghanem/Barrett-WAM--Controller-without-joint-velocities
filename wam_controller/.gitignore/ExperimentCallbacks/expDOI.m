function [experiment,callbackData] = expDOI(experiment, callbackData)
%EXPDOI Use the GPE as a planner
%   Required fields in callbackData.inputParams: 
%     objectid: the id of the object in the tray ex: '6.01'
    
% [exp, tformInfo, wamInfo, cal] = runExperiment('C:\Users\robotics\Documents\MATLAB\wam_controller\CameraCalibrations\Project20120520.ttp', 'expDOI', inputParams, tformInfo);

    % State labels
    DONE = 0;
    INITIALIZE = 1;
    SETUP_SENSORS = 2;
    RECIEVE_GRASP_PLAN = 3;
    MONITOR_WAM_STATE = 4;
    CLOSE_UNTIL_CONTACT = 5;
    CLEANUP   = 6;
    CLEANUP_1 = 7;
    CLEANUP_2 = 8;
    CLEANUP_3 = 9;
    CLEANUP_4 = 10;
    CLEANUP_5 = 11;

    switch(experiment.state)
        case INITIALIZE  % Initialization state
            % Connect to GPE
            %experiment.Connection = connect_to_GPE();
            
            % Homing gripper to closed position
            sendWSGHoming(callbackData.netInfo,'negative');
            
            % Locate the tray in the list of trackables
            if(~isfield(experiment,'objId'))
                experiment.objId = getTrackableIdByName(experiment.trackData.trackables, 'Tray');
                % check if tray exists
                if(experiment.objId == 0)
                    warning('There is no trackable named "Tray"');
                    experiment.state = CLEANUP_5;
                    return;
                end
            end
            
            % Plan for the current scenario
            planSuccess = DOI_Planning(experiment,callbackData);
            if ~planSuccess
               warning('There was an error during planning');
               experiment.state = CLEANUP_5;
               return;
            end

            experiment.record = 1;
            callbackData.firstWaypoint = [];
            
            % joint space move to ready configuration
            sendArmJspaceMove(callbackData.netInfo, callbackData.wamInfo.arm.ready, .5*ones(1,7), .5*ones(1,7));
            
            experiment.stateTime = callbackData.recvTime;
            experiment.state = SETUP_SENSORS;
            
            
        case SETUP_SENSORS
            % Wait until arm is in ready position, and at least 2 seconds have passed 
            if(sum(callbackData.wamDataRcvd.qdes == callbackData.wamInfo.arm.ready) == 7 && callbackData.recvTime - experiment.stateTime >= 2)
                experiment.stateTime = callbackData.recvTime;
                sendWSGTareForceSensor(callbackData.netInfo);
                pause(.1);
                sendWSGPosition(callbackData.netInfo,.11,.7);
                pause(.1);
                sendTareFTSensor(callbackData.netInfo);
                experiment.tareCount = 1;
                experiment.state = RECIEVE_GRASP_PLAN;
                fprintf('State [%d] RECIEVE_GRASP_PLAN\n',experiment.state);
            end
            
        case RECIEVE_GRASP_PLAN % Recieves and processes messages from the server
            % Read a message from the server
            Header = pnet(experiment.Connection,'Read',7,'uint8');
            if ~isempty(Header)
                [ID,Size] = parse_header(Header); % read and parse header
                fprintf('\nReceived message (ID: %d, Size: %d)\n',ID,Size);
                switch ID
                    case 1 % Plan Response
                        PlanFile = 'temp_plans.txt';
                        pnet(experiment.Connection, 'ReadToFile', PlanFile, Size-7);
                        fprintf('Saved grasp plan file (%s)\n',PlanFile);
                        % Message ID Enumeration
                        EXECUTE     = 2;
                                                
                        % Allow the user to select a 'good' grasp plan
                        graspPlanID = exTestGraspSetup(strcat(pwd,'\',PlanFile),experiment,callbackData);

                        if graspPlanID > 0
                            send_DOI_GPE(experiment.Connection, EXECUTE, graspPlanID);
                            fprintf('Sent grasp planID to GPE\n');
                        else
                            fprintf('User aborted experiment\n');
                            experiment.state = CLEANUP;
                        end
                        
                    case 2
                        % Execute the first waypoint
                        try
                            fprintf('Recieved waypoint from GPE\n');
                            % Clear the results from any previous moves
                            fprintf('Clearing any Result Packets left in buffer\n');
                            getWamExecutionResultPacket(callbackData.netInfo);
                            fprintf('Result Packets cleared\n');
                            experiment = DOI_Execution(experiment, callbackData, Size);
                            fprintf('Waypoint execution has begun\n');
                            experiment.state = MONITOR_WAM_STATE;
                            fprintf('State [%d] MONITOR_WAM_STATE\n',experiment.state);
                        catch e
                            fprintf('Error Executing move to waypoint\n%s\n%s',e.identifier,e.message);
                            experiment.state = CLEANUP;
                        end
                end
            else
                fprintf('Failed to hear back from the server!');
                experiment.state = CLEANUP;
            end
            
        case MONITOR_WAM_STATE % Monitors data coming back from the WAM and sends to GPE when necessary
            
            signals = [callbackData.wamDataRcvd.force'; callbackData.wamDataRcvd.torque'; callbackData.wamDataRcvd.WSGForce];
            state = 1;
            gripperconfig = getTransformN2Base(7,callbackData.wamInfo,callbackData.wamDataRcvd.q);
            [executionresult contactornot contactmode] = getWamExecutionResultPacket(callbackData.netInfo);
            %contactmode = callbackData.wamDataRcvd.contactscenario;
            
%             % Put this block back in to end experiment as soon as there is contact
%             if contactornot == 1
%                 fprintf('Contact detected. %d %d  Experiment is stopping\n',contactornot,contactmode);
%                 experiment.state = CLEANUP;
%                 return;
%             end
            
            if executionresult ~= -1 % We have an execution result, either success or failure (1 or 0)                
                
                % Send the status message to the GPE
                Message = packManipulationStatusMessage(state, signals, gripperconfig, executionresult, contactmode);
                pnet(experiment.Connection, 'Write', Message);
                
                fprintf('Sending Message to Server with result [%d]\n',executionresult);
                
                % Recieve the next waypoint command from the GPE
                pnet(experiment.Connection,'SetReadTimeout', 60);
                Header = pnet(experiment.Connection,'Read',7,'uint8');
                fprintf('Recieved next message\n');
                if ~isempty(Header)
                    [ID, Size] = parse_header(Header); % read and parse header
                    % fprintf('\nReceived message (ID: %d, Size: %d)\n',ID,Size);
                    if ID ~= 2
                        warning('invalid message from server!');
                        experiment.state = CLEANUP;
                    else
                        try
                            fprintf('Clearing Result Packets\n');
                            pause(1);
                            getWamExecutionResultPacket(callbackData.netInfo);
                            fprintf('Result Packets cleared\n');
%                             in = input('Press [Enter] to execute the waypoint. Enter [;] to cleanup. ','s');
%                             if strcmp(in,';')
%                                experiment.state = CLEANUP;
%                                return;
%                             end
                            experiment = DOI_Execution(experiment, callbackData, Size);
                        catch e
                            fprintf('Error Executing move to waypoint\n %s\n%s\n',e.identifier,e.message);
                            experiment.state = CLEANUP;
                        end
                    end
                else
                    fprintf('Timeout for hearing back from the server!\n');
                    fprintf('Execution completed!\n');
                    experiment.state = CLEANUP;
                end
            end
            
        case CLOSE_UNTIL_CONTACT
            
            dt = callbackData.recvTime - experiment.stateTime;
            
            [executionresult contactornot contactmode] = getWamExecutionResultPacket(callbackData.netInfo);
            
            % If the gripper is closed all the way, we have missed the object
%             if callbackData.wamDataRcvd.WSGPosition < .001
%                 callbackData.wamDataRcvd.WSGPosition
%                 fprintf('Missed the object\n');
%                 %TODO How to handle a missed object
%                 experiment.state = MONITOR_WAM_STATE;
%                 fprintf('State [%d] MONITOR_WAM_STATE\n',experiment.state);
%             
            if executionresult == 2
                  % Put this block back in to end experiment as soon as a contact is detected
%                 fprintf('Contact detected. %d %d  Experiment is stopping\n',contactornot,contactmode);
%                 sendWSGPosition(callbackData.netInfo,.11);
%                 pause(1);
%                 experiment.state = CLEANUP;
%                 return;

                fprintf('Contat %d detected',contactmode);
                res = input('If this is correct press [y]: ','s');
                if strcmp(res,'y') ~= 1
                    sendWSGPosition(callbackData.netInfo,.11);
                    pause(1);
                    experiment.state = CLEANUP;
                    return;
                end
                
                setWSGForceLimit(callbackData.netInfo,50);pause(1);
                % Contact scenario enumeration
                LEFT_FINGER_TOUCH = 7;
                RIGHT_FINGER_TOUCH = 8;
                if contactmode == LEFT_FINGER_TOUCH
                    finger = 'left';
                elseif contactmode == RIGHT_FINGER_TOUCH
                    finger = 'right';
                else
                    fprintf('Compliant close called, but contact scenario was %d not left finger(7) or right finger(8)',callbackData.wamDataRcvd.contactscenario);
                    %warning('Compliant close called, but contact scenario was %d not left finger(7) or right finger(8)',callbackData.wamDataRcvd.contactscenario);
                    experiment.state = CLEANUP_5;
                    return;
                end
                fprintf('Sending compliant close 2 with finger %s\n',finger);
                sendCompliantClose(callbackData.netInfo,0,finger);
                experiment.state = MONITOR_WAM_STATE;
                fprintf('State [%d] MONITOR_WAM_STATE\n',experiment.state);
                
            % if enough time has passed and there is force between the fingers, assume a good grasp
            elseif dt > 5 && callbackData.wamDataRcvd.WSGForce > 2
                fprintf('Assuming a successful close\n');
                experiment.state = MONITOR_WAM_STATE;
                fprintf('State [%d] MONITOR_WAM_STATE\n',experiment.state);
            end
            
        case CLEANUP
            fprintf('\nExperiment has terminated. WAM will be moved into arm_home_norest position\n');
            experiment.stateTime = callbackData.recvTime;
            experiment.state = CLEANUP_1;
            
        case CLEANUP_1
            if isfield(experiment,'firstWaypoint')
                % Wait a bit before doing anything
                if(callbackData.recvTime - experiment.stateTime >= 2.5)
                    fprintf('Sending to first waypoint\n');
                    %sendArmJspaceMove(callbackData.netInfo,experiment.firstWaypointQ);
                    sendArmCspaceMove(callbackData.netInfo,experiment.firstWaypoint,0);
                    experiment.state = CLEANUP_2;
                    experiment.stateTime = callbackData.recvTime;
                end
            else
                experiment.state = CLEANUP_2;
            end
            
        case CLEANUP_2
            % Wait to finish Cartesian space move, then move to arm_ready
            if(sum(isnan(callbackData.wamDataRcvd.wddes)) == 7)
                input('Press [Enter] to send arm to ready position and then home\n');
                sendArmJspaceMove(callbackData.netInfo,callbackData.wamInfo.arm.ready);
                experiment.state = CLEANUP_3;
                experiment.stateTime = callbackData.recvTime;
            end
            
        case CLEANUP_3
            % Wait to reach arm_ready then close gripper, and move home
            if(sum(callbackData.wamDataRcvd.qdes == callbackData.wamInfo.arm.ready) == 7)
                fprintf('Moving to home_no_rest\n');
                sendWSGPosition(callbackData.netInfo,0.01,.7);
                pause(.5);
                sendArmJspaceMove(callbackData.netInfo,callbackData.wamInfo.arm.home_norest);
                experiment.state = CLEANUP_4;
                experiment.stateTime = callbackData.recvTime;
            end
            
        case CLEANUP_4
            % stop when we get home
            if(sum(callbackData.wamDataRcvd.qdes == callbackData.wamInfo.arm.home_norest) == 7)
                experiment.state = CLEANUP_5;
            end
            
        case CLEANUP_5
            experiment.state = DONE;
            
    end

    function success = DOI_Planning(experiment,callbackData)
        %%
        Connection = experiment.Connection;

        success = true;

        PerceptionFile = 'CurrentExperiment.mat';

        Extents = [ -0.15  0.15
                    -0.4   0.4
                    -0.05  0.3 ];
        Resolution = 0.02;
        GridObj = occupancygrid(Extents,Resolution,gca);

        Folder = 'C:\Users\robotics\Documents\MATLAB\wam_controller\matlab_graspit_interface\matlab\src\scenarios\stl_files\';

        if(isempty(callbackData.trackDataRcvd))
           fprintf('Tracking data could not be found.\n');
           success = false;
           return
        else
           TWAMTray = callbackData.trackDataRcvd.trackables(experiment.objId).Tobjwam
        end

        Objects{1} = part(Folder,'tray.stl',gca);
        Objects{1}.Translation = TWAMTray(1:3,4);
        [theta, v] = tr2angvec(TWAMTray);
        % Watch for NaN in v
        if sum(isnan(v) == 1) > 0
           v = [0 0 1];
        end
        Objects{1}.Rotations = theta*v';

        TTrayObj = getTTrayObj(callbackData.inputParams.objectID);
        
        switch callbackData.inputParams.objectID
            case '5.02' % Brick (handle up)
                Objects{2} = graspobject('5.02','Brick with handle',Folder,'object_2_1_brick_w_handle.stl',gca);
        %         Objects{2}.Translation = [5.5;4;0.81]*0.0254;
            case '5.03' % Hat (open side up)
                Objects{2} = graspobject('5.03','Top hat',Folder,'object_2_2_top_hat.stl',gca);
        %         Objects{2}.Translation = [5;3;2.36]*0.0254;
            case '5.04' % Tumbler (open side up)
                Objects{2} = graspobject('5.04','Tumbler',Folder,'object_2_3_tumbler.stl',gca);
        %         Objects{2}.Translation = [1.64;1.63;3.04]*0.0254;
            case '5.01' % Block (holes up)
                Objects{2} = graspobject('5.01','Concrete block',Folder,'object_2_4_concrete_block.stl',gca);
        %         Objects{2}.Translation = [2.5;5;1.44]*0.0254;
            case '2.01' % Block
                Objects{2} = graspobject('2.01','Block',Folder,'object_1_2_block.stl',gca);
        end
        Objects{2}.Rotations = TTrayObj(1:3,1:3)*Objects{1}.Rotations;
        Objects{2}.Translation = Objects{1}.Translation + Objects{1}.Transform(1:3,1:3)*TTrayObj(1:3,4);

        %% Cleanup Objects
        for i = 1:numel(Objects)
            Objects{i}.Units = 'in';
            Objects{i}.FramesVisible = 'off';
        end

        %% Add Objects to Occupancy Grid
        for i = 1:numel(Objects)
            switch i
                case 1 % seafloor
                    Type = 3;
                case 2 % object of interest
                    Type = 1;
                otherwise % obstacles
                    Type = 2;
            end
            addpart(GridObj,Type,Objects{i})
        end

        %% Report Position of Object of Interest
        if numel(Objects) > 1
            fprintf('\nPosition of Object of Interest:\n')
            T = Objects{2}.Transform % transform from object to tray frame
            rph = R2rph(T(1:3,1:3),'b'); % roll, pitch, heading of object
            fprintf('\nTranslation (m):\t%f\t%f\t%f\n',T(1:3,4))
            fprintf('\nRotation (rad):\t\t%f\t%f\t%f\n',rph)
        end

        Grid = struct();
        Grid.Extents = GridObj.Extents;
        Grid.Resolution = GridObj.Resolution;
        Grid.Slices = GridObj.Slices;
        Grid.Matrix = GridObj.Matrix;

        Scene = struct();
        Scene.Index = 1;  % TODO Does the index matter?
        Scene.Description = 'Current Scenario';
        Scene.Grid = Grid;
        Scene.Objects = Objects;

        save(PerceptionFile,'Scene');

        % Send the perception file to the GPE
        %load(PerceptionFile,'Scene'); % the perception file (just to get the scenario index and description)

        send_DOI_GPE(Connection,1,PerceptionFile); % send the file to the GPE as a plan request message
        fprintf('\nSent request for grasp plans for scenario %d (%s)\n',Scene.Index, Scene.Description);

    end

    function experiment = DOI_Execution(experiment,callbackData, Size)
        Connection = experiment.Connection;
        payload = pnet(Connection,'Read',Size-7,'uint8');
        [waypoint, gripwidth, motionType, guarded, adjust_mode, compliant_close, squeeze ]...
        = interpretWaypointMessage(payload);

        % Store the first waypoint that we go to so we can use it to safely
        % cleanup the arm later.
        if ~isfield(experiment,'firstWaypoint')
            experiment.firstWaypoint = waypoint;
        elseif experiment.tareCount == 1
            %We have reached the first waypoint, and have the second, we should bias the FT sensor to make near zeros errors
            fprintf('Biasing FT Sensor in pregrasp pose\n');
            sendBiasFTSensor(callbackData.netInfo);
            experiment.tareCount = experiment.tareCount + 1;
            pause(1);
            fprintf('Done Biasing\n');
        end
        
        % Make sure the force limit is high if the squeeze flag is set
        if squeeze
            setWSGForceLimit(callbackData.netInfo,50);pause(.5);
        end

        FREEMOVE = 0;
        STRAIGHTLINE = 1;
        
        if compliant_close
            % Define Const in the gripper frame
            const = [1200;0;1200;300;300;300];
            
            sendArmImpedanceMove(callbackData.netInfo,waypoint,4,[0;0;0],const,[0;0;0],waypoint(1:3,1:3));
            pause(.01);
            sendWSGPosition(callbackData.netInfo,gripwidth);
%             direction = input('[+] to move in +Y direction. [-] to move in -Y direction. [Enter] to maintain: ','s');
%             T = eye(4);
%             if strcmp(direction,'+')
%                 T(2,4) =  0.09;
%             elseif strcmp(direction,'-')
%                 T(2,4) = -0.09;
%             end
%             sendArmCspaceMove(callbackData.netInfo,waypoint*T,1,.2,adjust_mode, squeeze);
            
            
%             if callbackData.wamDataRcvd.contactscenario == 0
%                 % Set a soft force limit so fingers will stop when moving to position
%                 fprintf('Moving fingers to grasping width\n');
%                 setWSGForceLimit(callbackData.netInfo,15);pause(1);
%                 sendWSGPosition(callbackData.netInfo,gripwidth,.05,'absolute','stop');
%                 experiment.stateTime = callbackData.recvTime;
%                 experiment.state = CLOSE_UNTIL_CONTACT;
%                 fprintf('State [%d] CLOSE_UNTIL_CONTACT\n',experiment.state);
%             else
%                 fprintf('Sending complant close 1\n');
%                 setWSGForceLimit(callbackData.netInfo,50);pause(1);
%                 LEFT_FINGER_TOUCH = 7;
%                 RIGHT_FINGER_TOUCH = 8;
%                 if contactmode == LEFT_FINGER_TOUCH
%                     finger = 'left';
%                 elseif contactmode == RIGHT_FINGER_TOUCH
%                     finger = 'right';
%                 else
%                     fprintf('Compliant close called, but contact scenario was %d not left finger(7) or right finger(8)',callbackData.wamDataRcvd.contactscenario);
%                     warning('Compliant close called, but contact scenario was %d not left finger(7) or right finger(8)',callbackData.wamDataRcvd.contactscenario);
%                     experiment.state = CLEANUP_5;
%                     return;
%                 end
%                 sendCompliantClose(callbackData.netInfo,gripwidth,finger);
%             end
            
        else % Not a compliant close
            if motionType == FREEMOVE % A jointspace move

                sendArmIKMove(callbackData.netInfo,waypoint);
                sendWSGPosition(callbackData.netInfo,gripwidth);

            elseif motionType == STRAIGHTLINE % A guarded or unguarded straightline
                
                sendArmCspaceMove(callbackData.netInfo,waypoint,guarded,.2,adjust_mode, squeeze);
                
                sendWSGPosition(callbackData.netInfo,gripwidth);
            end
        end

    end

    % Parses out a waypoint message into its components, the waypoint transform,
    % the gripper width and the type of movement to that waypoint
    function [waypoint, gripwidth, motionType, guarded, adjust_mode, compliant_close, squeeze] = interpretWaypointMessage(payload)
        HeaderSize = 7;

        waypoint = [double(reshape(typecast(payload(8-HeaderSize:55-HeaderSize), 'single'), 3, 4)); 0 0 0 1];
        gripwidth = double(typecast(payload(56-HeaderSize:59-HeaderSize), 'single'));
        waypointType = payload(60-HeaderSize);

        guarded         = double(bitshift(bitand(waypointType,uint8(bin2dec('10000000'))),-7)); % 0 - Unguarded, 1 - Guarded
        motionType      = double(bitshift(bitand(waypointType,uint8(bin2dec('01000000'))),-6)); % 0 - Free move, 1 - Straightline
        adjust_mode     = double(bitshift(bitand(waypointType,uint8(bin2dec('00110000'))),-4)); % 00 - Approach, 01 - Backup, 10 - Left/Right Adjust, 11 - Up/Down Adjust
        compliant_close = double(bitshift(bitand(waypointType,uint8(bin2dec('00000010'))),-1)); % 0 - Perform regular close, 1 - Perform compliant close
        squeeze         = double(bitshift(bitand(waypointType,uint8(bin2dec('00000001'))), 0)); % 0 - Do not squeeze the object, 1 - Squeeze the object
        
        fprintf('Waypoint Type %d\n',waypointType);
        fprintf(' Guarded %d\n Motion Type %d\n Adjust_Mode %d\n Compliant Close %d \n Squeeze %d\n',guarded,motionType,adjust_mode,compliant_close,squeeze);
    end

end
