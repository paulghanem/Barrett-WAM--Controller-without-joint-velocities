function sendArmImpedanceMove(netInfo,transfer, Movetime,poff,Const,Fp,R)
tf = Movetime;
% poff is offset to point of impedance in tool frame; transfer function
% stays the same
% Const is the diagonal elements of K (in world frame)
% Fp is end effector force (in world frame?)
% R rotation from world frame to tool frame



xf = transfer(1:3,4);
qf = Quaternion(transfer);
quatf=[qf.s qf.v]';
sendPacket(netInfo,tf,xf,quatf,poff,Const,Fp,R);

end
function sendPacket(netInfo,tf,xf , quatf,poff,Const,Fp,R)
    
    movetype = 5;

    if(~strcmp(class(tf(1)), 'double') || sum(size(tf') == [1 1]) ~= 2)
        error('invalid data format for q0');
    end
   
    if(~strcmp(class(xf(1)), 'double') || sum(size(xf') == [1 3]) ~= 2)
        error('invalid data format for xf');
    end
   
    if(~strcmp(class(quatf(1)), 'double') || sum(size(quatf') == [1 4]) ~= 2)
        error('invalid data format for quatf');
    end   
    
    if(~strcmp(class(R(1)),'double') || sum(size(R) == [3 3]) ~= 2)
        error('invalid data format for R');
    end
   
    data = [movetype tf' xf' quatf' poff' Const' Fp' R(1,1) R(1,2) R(1,3) R(2,1) R(2,2) R(2,3) R(3,1) R(3,2) R(3,3) zeros(1,719)];
    
    % send the packet
    sendArmMove(netInfo,data);
end