% intToBytes -> bytes = typecast(int32(259),'uint8')
% bytesToInt -> num = typecast(bytes,'int32')

% doubleToBytes -> bytes = typecast(3.1,'uint8')
% bytesToDouble -> num = typecast(bytes,'double')

function [wamData] = initWamControl(frameBuffSize)
%     xpcTarg = []; wamData = [];
%     return;
    
    % startup xPC target
    xpcTarg = xpctarget.xpc;
    %xpcTarg.targetping;
    if (strcmp(xpcTarg.Connected,'No'))
        error('Could not connect to xPC target machine');
    end
    
    %xpcTarg.load(wamModelFile);
    xpcTarg.start();
    
    wamData = struct('q',nan(frameBuffSize,7),'qd',nan(frameBuffSize,7),'qdes',nan(frameBuffSize,7),'qddes',nan(frameBuffSize,7),'wdes',nan(frameBuffSize,7),'wddes',nan(frameBuffSize,7),'tau',nan(frameBuffSize,7),'WSGForce',nan(frameBuffSize,1));
end
