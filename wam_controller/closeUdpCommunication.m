function closeUdpCommunication( netInfo )
%CLOSEUDPCOMMUNICATION Summary of this function goes here
%   Detailed explanation goes here

    netInfo.udpSock.close();
    netInfo.udpWSGSock.close();
    netInfo.udpExecutionResultSock.close();

end

