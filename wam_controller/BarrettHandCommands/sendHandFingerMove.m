function sendHandFingerMove(netInfo, q, fingerMask)
    % check for appropriate finger mask
    if(~ischar(fingerMask) || length(fingerMask) ~= 3)
        error('invalid finger mask, must be char of length 3');
    end
    
    % check that q is a scalar double
    if(~isscalar(q) || ~strcmp(class(q),'double'))
        error('q must be a scalar double');
    end
    
    global hand_m2jp;
    hand_j2mp = hand_m2jp([1 3 5 7],:)^-1;
    fing_j2mp = hand_j2mp(1,1);
    
    fing = (1:3)';
    fing = fing(logical(str2num(fingerMask')));
    fing_mpos = int32(fing_j2mp*q);
    
    % check that motor position is within proper bound
    if (fing_mpos > 20000)
        fing_mpos = 20000;
    elseif (fing_mpos < 0)
        fing_mpos = 0;
    end
    
    data = uint8([num2str(fing)' 'M ' num2str(fing_mpos)]);
    
    sendHandCommand(netInfo, data);
end
