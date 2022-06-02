function refreshWAMPackets( netInfo )
%REFRESHWAMPA Summary of this function goes here
%   Detailed explanation goes here

    succ = 1;

    while succ
       try
           getWamPacket(netInfo);
       catch e
           succ = 0;
       end
    end

end

