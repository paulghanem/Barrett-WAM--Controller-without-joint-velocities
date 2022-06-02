function runCleanupGracefully(netInfo,connection)
    fprintf('\nCleaning up gracefully...\n');
       
    try
        setWSGSetForcePeriod(netInfo,0);
        pause(1);
        setWSGPositionPeriod(netInfo,0);
        %WSGclearPackets(netInfo);
    catch e
    end
        
    try
        if(libisloaded('NPTrackingTools'))
            calllib('NPTrackingTools', 'TT_Shutdown');
        end
    catch e
    end
    
%     if (~isempty(udpSock))
%         udpSock.disconnect();
%         udpSock.close();
%     end
    try
        s = input('Press [A] to clean up automatically if WAM is in home position.\nPress [M] to clean up manually.\n','s');
        if strcmpi(s,'A');
            sendStopTorque(netInfo);
            input('\nPlease Shift-Idle the WAM (if necessary), then Press Enter...');
            xpcTarg = xpctarget.xpc;
            xpcTarg.stop();
        else
            fprintf('The simpliest way to finish the experiment is to follow these steps:\n');
            fprintf('  - Reconnect to the WAM\n');
            fprintf('     >> netInfo = initUdpCommunication();\n');
            fprintf('  - Send the arm a Joint Space move command to "arm_home_norest"\n');
            fprintf('     >> sendArmJSpaceMove(netInfo,arm_home_norest);\n');
            fprintf('  - Shift-Idle the WAM when it reaches the home position.\n');
            fprintf('  - Stop the xPC Target model by clicking on the xPC target shortcut on the top toolbar,\n    connecting to WAM_Target and pressing the Stop button\n');
            fprintf('  - Disconnect from the WAM and Shunk gripper\n');
            fprintf('     >> closeUdpCommunication(netInfo)\n');
        end
    catch e

    end
    
    try
        closeUdpCommunication(netInfo)
    catch e
        fprintf('UDP Connection not closed properly. %s\n',e.message);
    end
    
    try
       pnet(connection,'close'); 
    catch e
       fprintf('Could not close connection to GPE_Server. %s\n',e.message);
    end
    fprintf('\n');
end
