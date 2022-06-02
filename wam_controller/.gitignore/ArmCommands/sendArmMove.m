function sendArmMove(netInfo,values)
    % check that packet has appropriate length and type
    if(~strcmp(class(values(1)), 'double') || length(values) ~= 749)
        error('invalid data format for values');
    end

    % set the arm port
    ARM_PORT = 5555;
    
    % convert data to bytes
    data = typecast(values,'uint8');
    
    % send the packet
    netInfo.udpSock.send(java.net.DatagramPacket(data, length(data), netInfo.wamAddr, ARM_PORT));
end
