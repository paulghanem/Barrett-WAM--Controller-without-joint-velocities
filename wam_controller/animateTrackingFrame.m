% don't think this is quite finished
function fig = animateTrackingFrame(experiment,fidx,axlim,makefig)
    % set defaults
    if(nargin < 3)
        axlim = 1;
    end
    
    if(nargin < 4)
        makefig = 1;
    end
    
    % make a figure window if necessary
    if(makefig)
        fig = figure;
    else
        fig = nan;
    end
    
    
    % clear the figure window
    clf;
    
    % set background color to light gray
    lightGray = [0.9 0.9 0.9];
    set(gcf, 'Color', lightGray);
    
    % grab the rawMarkers
    rawMarkers = experiment.trackData.rawMarkers(:,:,fidx);
    rawMarkers = rawMarkers(~isnan(rawMarkers(:,1)),:);
    
    % grab the trackables
    trackableCount = length(experiment.trackData.trackables);
    trackableColors = 'rgbcmy';
    quivColors = 'rgb';
    numColors = length(trackableColors);
    
    % plot names and handles for the legend
    phandles = nan(1,trackableCount);
    pstrings = cell(1,trackableCount);
    
    % plot the trackables
    for tidx=1:trackableCount
        % grab this trackable
        trackable = experiment.trackData.trackables(tidx);
        pstrings{tidx} = [' ' trackable.name];
        
        % check if the trackable is visible
        if(trackable.lastTrack(fidx) == 0)
            % determine its color
            colorIdx = mod(tidx-1,numColors)+1;
            tcolor = trackableColors(colorIdx);

            % draw the trackable markers, connected
            mepos = [trackable.mepos(:,:,fidx); trackable.mepos(1,:,fidx)];
            phandles(tidx) = plot3(mepos(:,1),mepos(:,2),mepos(:,3),[tcolor 'o-'],'LineWidth',4); hold on;
            
            % remove the rawMarkers that correspond to this trackable
            markerCount = size(trackable.mbase,1);
            for midx=1:markerCount
                idx = find(trackable.mapos(midx,1,fidx)==rawMarkers(:,1) & trackable.mapos(midx,2,fidx)==rawMarkers(:,2) & trackable.mapos(midx,3,fidx)==rawMarkers(:,3));
                rawMarkers = removerows(rawMarkers,idx);
            end

            % show the trackable centroid and orientation
            quivStart = trackable.tpos(fidx,:);
            R = getQuaternion2Rot(trackable.tqtr);
            for qidx=1:3
                quivDir = 0.1*R(:,qidx);
                quiver3(quivStart(1),quivStart(2),quivStart(3),quivDir(1),quivDir(2),quivDir(3),quivColors(qidx),'LineWidth',2); hold on;
            end
        else
            % make a blank plot with the same color as the background
            % so the label doesn't show up in the legend
            phandles(tidx) = plot3(nan,nan,nan,'o-','LineWidth',4,'Color',lightGray); hold on;
        end
        
        % print the trackable name on the graph with its color
    end
    
    % plot the arm
    
    
    % show the legend
    lh = legend(phandles,pstrings); legend('boxoff');
    set(lh,'FontSize',15);
    set(lh,'FontWeight','bold');
    
    % plot the raw markers
    plot3(rawMarkers(:,1),rawMarkers(:,2),rawMarkers(:,3),'ko','LineWidth',4); hold on;
    
    % show the axis labels and grid, set the axis limits, then draw now
    % with the xlabel,ylabel,zlabel or pause time less than 0.1 this freezes in real time mode
    axis([-axlim axlim -axlim axlim -axlim axlim]); grid;
    xlabel('x'); ylabel('y'); zlabel('z');
    drawnow;
    pause(0.1);
end
