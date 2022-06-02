% returns 0 if trackable was not found
function id = getTrackableIdByName(trackables, name)
    % locate the trackable in the list
    id = 0;
    for tidx=1:length(trackables)
        if(strcmp(trackables(tidx).name, name))
            id = tidx;
        end
    end
end
