function [data, recvTime, blockNum] = getWamPacket(netInfo)
    % receive the 484 byte packet over UDP (4 bytes for blockNum, 480 bytes for dbls)
    pktData = zeros(1,484,'uint8');
    packet = java.net.DatagramPacket(pktData, length(pktData));
    netInfo.udpSock.receive(packet);
    
    % grab the system time
    recvTime = java.lang.System.nanoTime()/1e9;
    rawData = typecast(int8(packet.getData()),'uint8');
    
    % extract the raw data
    blockNum = typecast(rawData(1:4),'uint32');
    dbls = typecast(rawData(5:end),'double');
  
    % extract the WAM data
    wdes = dbls(1:7)';
    wddes = dbls(8:14)';
    qdes = dbls(15:21)';
    qddes = dbls(22:28)';
    q = dbls(29:35)';
    qd = dbls(36:42)';
    force = dbls(43:45)';
    torque = dbls(46:48)';
    WSGForce = dbls(49);
    WSGPosition = dbls(50);
    % GraspSuccess = dbls(51);
    tau = dbls(52:58)';
    contactornot = dbls(59);
    contactscenario = dbls(60);
    
    % save it in a struct
    data = struct('wdes',wdes,'wddes',wddes,'qdes',qdes,'qddes',qddes,'q',q,'qd',qd,'tau',tau,...
                  'force',force,'torque',torque,'WSGForce',WSGForce,'WSGPosition',WSGPosition,...
                  'contactornot',contactornot,'contactscenario',contactscenario);
end