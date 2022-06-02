function [netInfo] = initUdpCommunication()
%     import java.net.*;
    WAM_LOCAL_PORT = 3333;  % Port that WAM Data comes in on
    WAM_EXECUTION_RESULT_LOCAL_PORT = 3334; % Port that execution result data comes in on
    WSG_LOCAL_PORT = 1501;  % Port that Gripper Data comes in on

    % create java objects for UDP
    udpSock = java.net.DatagramSocket(WAM_LOCAL_PORT, java.net.InetAddress.getByName('192.168.1.11'));
    wamAddr = java.net.InetAddress.getByName('192.168.1.10');
%     udpSock.setReuseAddress(true);
    udpSock.setSoTimeout(1000);
%     udpSock = DatagramSocket(LOCAL_PORT, InetAddress.getByName('10.0.2.15'));
%     wamAddr = InetAddress.getByName('10.0.2.15');

    udpExecutionResultSock = java.net.DatagramSocket(WAM_EXECUTION_RESULT_LOCAL_PORT, java.net.InetAddress.getByName('192.168.1.11'));
    udpExecutionResultSock.setSoTimeout(50);
 
    %udpWSGSock = java.net.DatagramSocket(WSG_LOCAL_PORT, java.net.InetAddress.getByName('192.168.1.11'));
    %WSGAddr = java.net.InetAddress.getByName('192.168.1.20');
    udpWSGSock = java.net.DatagramSocket(WSG_LOCAL_PORT, java.net.InetAddress.getByName('192.168.1.11'));
    WSGAddr = java.net.InetAddress.getByName('192.168.1.20');
    udpWSGSock.setSoTimeout(10);

    netInfo = struct('udpSock',udpSock,'wamAddr',wamAddr,'udpWSGSock',udpWSGSock,'WSGAddr',WSGAddr, ...
                     'udpExecutionResultSock',udpExecutionResultSock);
    %netInfo = struct('udpWSGSock',udpWSGSock,'WSGAddr',WSGAddr);
end

