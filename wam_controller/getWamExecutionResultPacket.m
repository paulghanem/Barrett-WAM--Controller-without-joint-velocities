function [ executionResult contactOrNot contactScenario ] = getWamExecutionResultPacket( netInfo )
%GETWAMEXECUTIONRESULTPACKET Gets an execution result packt from the WAM
%   Returns the value in the packet.  If there was no packet (receive times out)
%   then return -1.

    persistent last_packet_type;
    
    executionResult = -1;
    last_execution_result = -1;
    contactOrNot = -1;
    last_contactOrNot = -1;
    contactScenario = -1;
    last_contactScenario = -1;
    last_FTError = -1;
        
    if isempty(last_packet_type)
       last_packet_type = -1; 
    end
    
    done = false;
    
    while ~done
        [startOrFinish currExecutionResult currContactOrNot curContactScenario currFTError t] = getPacket(netInfo);
        if startOrFinish == -1
           done = true;
        elseif startOrFinish == 1 % If this was a starting packet
           last_packet_type = startOrFinish
           fprintf('T: %.3f\n',t);
        elseif startOrFinish == 2 % If this was a finish packet
           last_packet_type = startOrFinish
           last_execution_result = currExecutionResult;
           last_contactOrNot = currContactOrNot;
           last_contactScenario = curContactScenario;
           last_FTError = currFTError;
           fprintf('T: %.3f\n',t);
        end
    end
    
    if last_packet_type == 1
       executionResult = -1;
       contactOrNot = -1;
       contactScenario = -1;
    elseif last_packet_type == 2
       executionResult = last_execution_result;
       contactOrNot = last_contactOrNot;
       contactScenario = last_contactScenario;
       FTError = last_FTError;
    end
    
    
    if contactOrNot ~= -1 || contactScenario ~= -1 || executionResult ~= -1
       executionResult
       contactOrNot
       contactScenario
    end
    
    % grab the system time
    %recvTime = java.lang.System.nanoTime()/1e9;
end

function [startOrFinish executionResult contactOrNot contactScenario FTError t] = getPacket(netInfo)

    % receive the 48 byte packet over UDP
    % 8 bytes (1 double) start or finish
    % 8 bytes (1 double) for ExecutionResult
    % 8 bytes (1 double) for FTError
    % 8 bytes (1 double) for contactOrNot
    % 8 bytes (1 double) for contactScenario
    % 8 bytes (1 double) for time
    pktData = zeros(1,48,'uint8');
    packet = java.net.DatagramPacket(pktData, length(pktData));

    try
        netInfo.udpExecutionResultSock.receive(packet);
    catch e
        startOrFinish = -1;
        executionResult = -1;
        contactOrNot = -1;
        contactScenario = -1;
        FTError = -1;
        t = -1;
        return;
    end

    % Unpack the data
    rawData = typecast(int8(packet.getData()),'uint8');
    startOrFinish = typecast(rawData(1:8),'double');
    executionResult = typecast(rawData(9:16),'double');
    FTError = typecast(rawData(17:24),'double');
    contactOrNot = typecast(rawData(25:32),'double');
    contactScenario = typecast(rawData(33:40),'double');
    t = typecast(rawData(41:48),'double');
end