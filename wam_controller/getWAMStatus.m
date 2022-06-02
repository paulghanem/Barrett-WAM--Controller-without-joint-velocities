function status = getWAMStatus( netInfo )
%GETWAMSTATUS Summary of this function goes here
%   Detailed explanation goes here
    
    % set the status port
    STATUS_PORT = 3334;
    % convert data to bytes
    data = typecast(1,'uint8');
    % send the packet
    netInfo.udpStatusSock.send(java.net.DatagramPacket(data, length(data), netInfo.statusAddr, STATUS_PORT));

    %  Recive the packet
    pktData = zeros(1,4,'uint8');
    packet = java.net.DatagramPacket(pktData, length(pktData));
    netInfo.udpStatusSock.receive(packet);
    % extract the raw data
    status = typecast(rawData(1:4),'uint32');
    
end

