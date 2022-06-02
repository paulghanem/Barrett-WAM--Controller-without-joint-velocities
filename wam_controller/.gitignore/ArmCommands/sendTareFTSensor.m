function sendTareFTSensor( netInfo )
%SENDTAREFTSENSOR Sends a tare command to the force torque sensor
%   Must wait two seconds before valid force/torque data is coming from the sensor

    % set the command port
    COMMAND_PORT = 6666;

    data = uint8([1 0 0 0 0 0 0 0 0]);
    
    % send the packet
    netInfo.udpSock.send(java.net.DatagramPacket(data, length(data), netInfo.wamAddr, COMMAND_PORT));
end

