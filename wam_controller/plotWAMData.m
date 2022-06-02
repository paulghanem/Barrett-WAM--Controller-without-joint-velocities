function plotWAMData(WAMDataFileName,wamInfo,t0,t1,plotJointAngles,plotFT, plotErrors, gripperInfo, filename)

    load(WAMDataFileName);

    if(t1*500 > length(t))
        t1 = length(t)/500;
    end
    
    % Convert to indexes
    t0 = uint32(t0*500);
    t1 = uint32(t1*500);
    
    if(plotJointAngles == 1)
        for i = 1:7
           figure(i)
           subplot(3,1,1)
           hold on
           plot(t(t0:t1),Q(t0:t1,i),'b');
           plot(t(t0:t1),Qdes(t0:t1,i),'r');
           legend('q','qdes');
           subplot(3,1,2)
           hold on
           plot(t(t0:t1),Qd(t0:t1,i),'b');
           plot(t(t0:t1),Qddes(t0:t1,i),'r');
           legend('qd','qddes');
           subplot(3,1,3)
           hold on
           plot(t(t0:t1),Qdd(t0:t1,i),'b');
           plot(t(t0:t1),Qdddes(t0:t1,i),'r');
           %axis([double(t0/500) double(t1/500) -.6 .6]);
        end
        
        % Plot the type
        figure()
        plot(t(t0:t1),Type(t0:t1));
    end
    
    if(gripperInfo == 1)
       figure()
       plot(t(t0:t1),GripperForce(t0:t1));
       title('Gripper Force');
       xlabel('Time (s)');
       ylabel('Force (N)');
       
       figure()
       plot(t(t0:t1),GripperPosition(t0:t1));
       title('Gripper Width');
       xlabel('Time (s)');
       ylabel('Position (m)');
    end
    
%     figure(16)
%     plot(t(t0:t1),Type(t0:t1));
%     title('Type');
    
    if(plotFT)
        
        % Allocate space for cartesian space positions and velocities
        % Each time step is held as a 4x4 transformation matrix from the WAM frame
        K = zeros(4,4,length(Q));
        Kdes = zeros(4,4,length(Q));
        FExp = zeros(3,length(Q));
        FExpDyn = zeros(3,length(Q));
        FAct = zeros(3,length(Q));
        FActOldFilter = zeros(3,length(Q));
        FActNewFilter = zeros(3,length(Q));
        TauActOldFilter = zeros(3,length(Q));
        TauActNewFilter = zeros(3,length(Q));
        TauExp = zeros(3,length(Q));
        TauExpDyn = zeros(3,length(Q));
        TauAct = zeros(3,length(Q));
        DisVec = zeros(3, length(Q));
        DisVecOldFilter = zeros(3, length(Q));
        DisVecNewFilter = zeros(3, length(Q));
        g = [0;0;-9.81]; % Gravity
        %r = [0;0;.06];  % Distace to COM of hand in the F/T sensors frame

    %   CALIBRATION WITH WIRES ATTACHED TO GRIPPER
    %     r = [0.003115;	0.005288;	0.053531];
    %     mass = 12.954349/9.81;     % Mas in kg
    %     bias =   [-15.7025;
    %                -0.9607;
    %                -0.1655;
    %                 0.0006;
    %                -0.6743;
    %                 0.0729;];
    %     C  =   [0.8572    0.0312    0.0259         0         0         0;
    %            -0.0350    0.9127    0.0214         0         0         0;
    %            -0.0064    0.0162    0.9816         0         0         0;
    %                  0         0         0    1.0000         0         0;
    %                  0         0         0         0    1.0000         0;
    %                  0         0         0         0         0    1.0000;];

    % %   CALIBRATION WITH WIRES UNATTACHED TO GRIPPER, CONTINIOUS MOVEMENT
    %     r = [0.001445;	0.000138;	0.053197];
    %     mass = 13.045551/9.81;
    %     bias = [-11.9478;
    %             7.4964;
    %            -0.4764;
    %            -0.4104;
    %            -0.5684;
    %             0.0210;];
    %         
    %     C = [ 0.8858    0.0257    0.0226         0         0         0;
    %          -0.0330    0.9172   -0.0156         0         0         0;
    %          -0.0036   -0.0073    0.9898         0         0         0;
    %                0         0         0    1.0000         0         0;
    %                0         0         0         0    1.0000         0;
    %                0         0         0         0         0    1.0000;];

        % CALIBRATION WITH MANY STEPS AND DATA ONLY TAKEN WHEN HAND IS STATIONARY
        r = [0.001127;	-0.000211;	0.054851];
        mass = 13.470174/9.81;
        bias = [-3.0001;
                -0.4735;
                 0.1594;
                -0.0170;
                -0.0556;
                 0.0013];

        C = [0.8615    0.0408    0.0453         0         0         0;
            -0.0423    0.8957    0.0443         0         0         0;
             0.0013   -0.0150    0.9900         0         0         0;
                  0         0         0    1.0000         0         0;
                  0         0         0         0    1.0000         0;
                  0         0         0         0         0    1.0000];

        %bias = zeros(6,1);  % Bias is taken care of in the controller

        for i = 1:length(Qdes)
            K(:,:,i) = getTransformN2Base(7,wamInfo,Q(i,:));
            Kdes(:,:,i) = getTransformN2Base(7,wamInfo,Qdes(i,:));
            R0Fg = K(1:3,1:3,i)'*g;
            FExp(:,i) = R0Fg*mass;
            TauExp(:,i) = cross(r,FExp(:,i));
            FExpDyn(:,i) = -SensorExpFT(i,1:3)';
            TauExpDyn(:,i) = -SensorExpFT(i,4:6)';
            FAct(:,i) = C(1:3,1:3)*(Force(i,:)' - bias(1:3));
            TauAct(:,i) = C(4:6,4:6)*(Torque(i,:)' - bias(4:6));
            FActOldFilter(:,i) = C(1:3,1:3)*(ForceOriginalFilter(i,:)' - bias(1:3));
            TauActOldFilter(:,i) = C(4:6,4:6)*(TorqueOriginalFilter(i,:)' - bias(4:6));
            FActNewFilter(:,i) = C(1:3,1:3)*(ForceNewFilter(i,:)' - bias(1:3));
            TauActNewFilter(:,i) = C(4:6,4:6)*(TorqueNewFilter(i,:)' - bias(4:6));

            if mod(i,5000) == 0
                i
            end
        end
        FerrOldFilter = FExp - FActOldFilter;
        TauErrOldFilter = TauExp - TauActOldFilter;
        FerrNewFilter = FExp - FActNewFilter;
        TauErrNewFilter = TauExp - TauActNewFilter;
        Ferr = FExp - FAct;
        TauErr = TauExp - TauAct;

        FerrOldFilterDyn = FExpDyn - FActOldFilter;
        TauErrOldFilterDyn = TauExpDyn - TauActOldFilter;
        FerrNewFilterDyn = FExpDyn - FActNewFilter;
        TauErrNewFilterDyn = TauExpDyn - TauActNewFilter;
        FerrDyn = FExpDyn - FAct;
        TauErrDyn = TauExpDyn - TauAct;

        for i=1:length(Qdes)
            DisVec(:, i) = inverse_cross_product(Ferr(:,i), TauErr(:,i));
            DisVecOldFilter(:, i) = inverse_cross_product(FerrOldFilter(:,i), TauErrOldFilter(:,i));
            DisVecNewFilter(:, i) = inverse_cross_product(FerrNewFilter(:,i), TauErrNewFilter(:,i));
        end

        n = (double(t1) - double(t0));

    %     FMaxErr = max(Ferr(:,t0:t1)')
    %     FMinErr = min(Ferr(:,t0:t1)')
    %     FErrAvg = sum(Ferr(:,t0:t1)')/n
    %     TauMaxErr = max(TauErr(:,t0:t1)')
    %     TauMinErr = min(TauErr(:,t0:t1)')
    %     TauErrAvg = sum(TauErr(:,t0:t1)')/n
    %     SSE = sum(sum( (Ferr(:,t0:t1)').^2 + (TauErr(:,t0:t1)').^2))/n


        
        namestrings = ['x'; 'y'; 'z'];
        hfig = figure(12);
        set(hfig, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
        for i = 1:3
            subplot(3,2,i*2-1);
            hold on
            h = plot(t(t0:t1),FAct(i,t0:t1),'r');
            plot(t(t0:t1),FExp(i,t0:t1),'b');
            plot(t(t0:t1),FActOldFilter(i,t0:t1),'g');
            plot(t(t0:t1),FActNewFilter(i,t0:t1),'k');
            plot(t(t0:t1),FExpDyn(i,t0:t1),'m');
            h_legend=legend('Force Actual Raw','Force Expected','Force Actual Old Filter','Force Actual New Filter','Force Expected Dynamics');
            set(h_legend,'FontSize',7);
            title(['FT/Sensor - Force ' namestrings(i, :)]);
            xlabel('Time (s)');
            ylabel('Force (N)');
            
            if exist('filename', 'var')
                x = get(h,'XData'); 		% Get the plotted data
                y = get(h,'YData');
                imax = find(max(abs(y)) == abs(y));
                text(x(imax),y(imax), filename,...
                    'VerticalAlignment','bottom',...
                    'HorizontalAlignment','right',...
                    'FontSize',8)
            end
        end
        for i = 1:3
            subplot(3,2,i*2);
            hold on
            h = plot(t(t0:t1),TauAct(i,t0:t1),'r');
            plot(t(t0:t1),TauExp(i,t0:t1),'b');
            plot(t(t0:t1),TauActOldFilter(i,t0:t1),'g');
            plot(t(t0:t1),TauActNewFilter(i,t0:t1),'k');
            plot(t(t0:t1),TauExpDyn(i,t0:t1),'m');
            h_legend = legend('Torque Actual Raw','Torque Expected','Torque Actual Old Filter','Torque Actual New Filter','Torque Expected Dynamics');
            set(h_legend,'FontSize',7);
            title(['FT/Sensor - Torque ' namestrings(i, :)]);
            xlabel('Time (s)');
            ylabel('Torque (Nm)');
            
             if exist('filename', 'var')
                x = get(h,'XData'); 		% Get the plotted data
                y = get(h,'YData');
                imax = find(max(abs(y)) == abs(y));
                text(x(imax),y(imax), filename,...
                    'VerticalAlignment','bottom',...
                    'HorizontalAlignment','right',...
                    'FontSize',8)
            end
        end
                
        hfig = figure(14);
        set(hfig, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
        for i = 1:3
            subplot(3,2,2*i-1);
            hold on
            h = plot(t(t0:t1),Ferr(i,t0:t1),'r');
            plot(t(t0:t1),FerrOldFilter(i,t0:t1),'g');
            plot(t(t0:t1),FerrNewFilter(i,t0:t1),'k');
            plot(t(t0:t1),FerrOldFilterDyn(i,t0:t1),'c');
            plot(t(t0:t1),FerrNewFilterDyn(i,t0:t1),'m');
            plot(t(t0:t1),FerrDyn(i,t0:t1),'y');
            title(['FT/Sensor - Error - Force ' namestrings(i, :)])
            h_legend = legend('Raw', 'Old filter', 'New Filter');
            set(h_legend,'FontSize',7);
            xlabel('Time (s)');
            ylabel('Force Error (N)');
             if exist('filename', 'var')
                x = get(h,'XData'); 		% Get the plotted data
                y = get(h,'YData');
                imax = find(max(abs(y)) == abs(y));
                text(x(imax),y(imax), filename,...
                    'VerticalAlignment','bottom',...
                    'HorizontalAlignment','right',...
                    'FontSize',8)
            end
        end
        for i = 1:3
            subplot(3,2,2*i);
            hold on
            h = plot(t(t0:t1),TauErr(i,t0:t1),'r');
            plot(t(t0:t1),TauErrOldFilter(i,t0:t1),'g');
            plot(t(t0:t1),TauErrNewFilter(i,t0:t1),'k');
            plot(t(t0:t1),TauErrOldFilterDyn(i,t0:t1),'c');
            plot(t(t0:t1),TauErrNewFilterDyn(i,t0:t1),'m');
            plot(t(t0:t1),TauErrDyn(i,t0:t1),'y');
            title(['FT/Sensor - Error - Torque ' namestrings(i, :)])
            h_legend = legend('Raw', 'Old filter', 'New Filter');
            set(h_legend,'FontSize',7);
            xlabel('Time (s)');
            ylabel('Torque Error (Nm)');
            if exist('filename', 'var')
                x = get(h,'XData'); 		% Get the plotted data
                y = get(h,'YData');
                imax = find(max(abs(y)) == abs(y));
                text(x(imax),y(imax), filename,...
                    'VerticalAlignment','bottom',...
                    'HorizontalAlignment','right',...
                    'FontSize',8)
            end
        end  
        
        hfig = figure(16);
        set(hfig, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
        hold on
        h = plot(t(t0:t1), GripperForce(t0:t1), 'b');
        title('Gripper Force')
        xlabel('Time (s)')
        ylabel('Force (N)')
        if exist('filename', 'var')
                x = get(h,'XData'); 		% Get the plotted data
                y = get(h,'YData');
                imax = find(max(abs(y)) == abs(y));
                text(x(imax),y(imax), filename,...
                    'VerticalAlignment','bottom',...
                    'HorizontalAlignment','right',...
                    'FontSize',8)
        end
         
        hfig = figure(18);
        set(hfig, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
        for i = 1:3
            subplot(1,3,i);
            hold on
            h = plot(t(t0:t1),DisVec(i,t0:t1) * 1000,'r');
            plot(t(t0:t1),DisVecOldFilter(i,t0:t1) * 1000,'g');
            plot(t(t0:t1),DisVecNewFilter(i,t0:t1) * 1000,'k');
            title(['Displacement Vector (Defined in Force-torque sensor frame): ' namestrings(i, :)])
            h_legend = legend('Raw', 'Old filter', 'New Filter');
            set(h_legend,'FontSize',7);
            xlabel('Time (s)');
            ylabel('Distance (mm)');
            v = axis();
            axis([v(1) v(2) -300 300]);
             if exist('filename', 'var')
                x = get(h,'XData'); 		% Get the plotted data
                y = get(h,'YData');
                imax = find(max(abs(y)) == abs(y));
                text(x(imax),y(imax), filename,...
                    'VerticalAlignment','bottom',...
                    'HorizontalAlignment','right',...
                    'FontSize',8)
            end
        end
    end
    
    if plotErrors
        figure(8)
        subplot(3,1,1);
        hold on
        plot(t(t0:t1),1000*squeeze(K(1,4,t0:t1)),'color',[1 0 0]);
        plot(t(t0:t1),1000*squeeze(Kdes(1,4,t0:t1)),'color',[.6 0 0]);

        subplot(3,1,2);
        hold on
        plot(t(t0:t1),1000*squeeze(K(2,4,t0:t1)),'color',[1 0 0]);
        plot(t(t0:t1),1000*squeeze(Kdes(2,4,t0:t1)),'color',[.6 0 0]);
        legend('y1','y1des');
        subplot(3,1,3);
        hold on
        plot(t(t0:t1),1000*squeeze(K(3,4,t0:t1)),'color',[1 0 0]);
        plot(t(t0:t1),1000*squeeze(Kdes(3,4,t0:t1)),'color',[.6 0 0]);
        legend('z1','z1des');

        figure(9)
        names = ['x'; 'y'; 'z'];
        for i=1:3
            % Plot the error
            subplot(3, 2, i*2-1);
            title('controller error')
            hold on
            plot(t(t0:t1),1000*squeeze(Kdes(i,4,t0:t1)-K(i,4,t0:t1)),'color',[.6 0 0])
            %legend('option1');
            xlabel('Time (second)');
            ylabel(['Tooltip ' names(i) '-Position (mm)']);

            % Plot the absolute error value
            subplot(3, 2, 2*i);
            title('controller error')
            hold on
            plot(t(t0:t1),1000*abs(squeeze(Kdes(i,4,t0:t1)-K(i,4,t0:t1))),'color',[.6 0 0])
            %legend('option1');
            xlabel('Time (second)');
            ylabel(['Tooltip ' names(i) '-Position (mm)']);
        end

        figure(10)
        plot(t(t0:t1),1000*sqrt(sum(abs(squeeze(Kdes(1:3,4,t0:t1)-K(1:3,4,t0:t1))).^2,1)),'r');
        title('distance error')
        ylabel('error (mm) ');
        xlabel('Time (second)');

        figure(11)
        a = 1000*squeeze(K(1,4,t0:t1));b= 1000*squeeze(K(2,4,t0:t1));c = 1000*squeeze(K(3,4,t0:t1));
        plot3(a, b, c,'color',[.6 0 0]);
        a = 1000*squeeze(Kdes(1,4,t0:t1));b= 1000*squeeze(Kdes(2,4,t0:t1));c = 1000*squeeze(Kdes(3,4,t0:t1));
        hold on
        plot3(a, b, c,'color','r');
        title('tooltip trajectory')
        
        figure()
        hold on
        plot(t(t0:t1),Error(t0:t1,1),'r');
        plot(t(t0:t1),Error(t0:t1,2),'g');
        plot(t(t0:t1),Error(t0:t1,3),'b');
        legend('x','y','z');
        title('Cartesian space error (used to drive Jacobian Pseudoinverse Controlller)');
    end
end



% if a x b = c, a, b are perpendicular 3*1 vectors, 
% given a, c, this function will return b
function [ a ] = inverse_cross_product(b, c)
a = cross(b, c)/dot(b(:)', b(:));
end

