function sendStopTorque( netInfo, duration )
%SENDTAREFTSENSOR Sends a message to the controller to start applying the calculated torque
%  duration (optional) the amount of time it takes for full torque to be applied

    % set the command port
    COMMAND_PORT = 6666;

    if nargin < 2
      duration = 1;
    end
    
    duration = ceil(duration);
    
    data = uint8([3 duration 0 0 0 0 0 0 0]);
    
    % send the packet
    netInfo.udpSock.send(java.net.DatagramPacket(data, length(data), netInfo.wamAddr, COMMAND_PORT));
end

