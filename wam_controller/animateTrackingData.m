% should use "movie" for this to speed it up
% don't think this is quite finished
function [fig,mov] = animateTrackingData(experiment, frameSteps)
    if(nargin < 2)
        frameSteps = 10;
    end

    frameCount = experiment.frameCount;
    
    fig = figure;
    c = onCleanup(@()close(fig));
    
    midx = 1;
    for fidx=1:frameSteps:frameCount
        animateTrackingFrame(experiment,fidx,1,0);
        mov(midx) = getframe;
        midx = midx+1;
    end
end
