function sendHandCommand(netInfo, data)
    % data length must be maximum 31 to leave space for the \r
    if (~strcmp(class(data(1)), 'uint8') || length(data) > 31)
        error('data length must be 31');
    end
    
    % pack data so it is length 32
    data = uint8([data 32*ones(1,31-length(data)) 13]);

    % set the hand port
    HAND_PORT = 4444;
    
    netInfo.udpSock.send(java.net.DatagramPacket(data, length(data), netInfo.wamAddr, HAND_PORT));
end
