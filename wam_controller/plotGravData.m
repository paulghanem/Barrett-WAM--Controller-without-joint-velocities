
% Beta  Angle of incline for box (degrees for now).  Should be determined via vision data.
% t     Time stamp data
% Q     Joint angles
% Qd    Joint velocities 
% Qdd   Joint accelerations 
% ForceTorque   Force-Torque data from wrist
% wamInfo       WAM information for things like transformations 
function plotGravData( Beta, t,Q,Qd,Qdd,ForceTorque,wamInfo )
Beta =  - Beta / (180/pi); % Convert to radians.  Note the negative (the box is being rotated in the neg direction).
T = size(t,1);

footPosition = zeros(3,T);
footVelocity = zeros(3,T);
footNormal = zeros(3,T);
footTrac = zeros(3,T); 
Fn = zeros(T,1);    % Normal force magnitude
Ft = zeros(T,1);    % Traction force magnitude
vt = zeros(T,1);    % Foot velocity in the negative traction direction

currentFootTransform = getTransformN2Base(7,wamInfo,Q(1,:));
footPosition(:,1) = currentFootTransform(1:3,4);

Vx = [1;0;0];
Vy = [0;1;0];
Vz = [0;0;1];
normDirection = Vz*cos(Beta)+cross(Vy,Vz)*sin(Beta)+Vy*dot(Vy,Vz)*(1-cos(Beta))
tracDirection = (-Vx)*cos(Beta)+cross(Vy,(-Vx))*sin(Beta)+Vy*dot(Vy,(-Vx))*(1-cos(Beta))
for i = 2:T
   currentFootTransform = getTransformN2Base(7,wamInfo,Q(i,:));
   footNormal(:,i) = currentFootTransform(1:3,1:3) * Vz; 
   footTrac(:,i) = currentFootTransform(1:3,1:3) * Vx; 
   footPosition(:,i) = currentFootTransform(1:3,4);
   footVelocity(:,i) = (footPosition(:,i) - footPosition(:,i-1))/.002;
   
   % Measurable parameters
    % Force normal to the terrain 
    %Fna(i) = -ForceTorque(i,3); 
    Fn(i) = dot( footNormal(:,i), tracDirection ) * ForceTorque(i,1) + ... 
            dot( footNormal(:,i), cross(normDirection,tracDirection) ) * ForceTorque(i,2) + ... 
            dot( footNormal(:,i), normDirection ) * ForceTorque(i,3);

    % Force in traction direction (in this experiment, the x-direction) 
    %Ft(i) = ForceTorque(i,1);  % Should really depend on the orientation of the box and foot
    Ft(i) = dot( tracDirection, footTrac(:,i) ) * ForceTorque(i,1) + ...
            dot( tracDirection, cross(footNormal(:,i), footTrac(:,i)) ) * ForceTorque(i,2) + ...
            dot( tracDirection, footNormal(:,i) ) * ForceTorque(i,3);

    % Foot velocity in the negative traction direction
    vt(i) = dot( footVelocity(:,i), tracDirection );  % -tracDirection
end

%figure(6); hold on;
%plot(t, Fna,t, Fn,'--');

figure(1);
subplot(3,1,1); hold on;
plot(t,Fn);   ylabel('F_n');
subplot(3,1,2); hold on;
plot(t,Ft);   ylabel('F_t');
subplot(3,1,3); hold on;
plot(t,vt);   ylabel('v_t');  xlabel('Time'); 

figure(2);  colr = rand(1,3);   hold on;  grid on;
plot3(vt,Ft,Fn,'o','MarkerEdgeColor',colr,'MarkerFaceColor',colr,'MarkerSize',2);
xlabel('vt'); ylabel('Ft'); zlabel('Fn'); 
view(2);

figure(9); hold on; grid on;
plot(vt,Ft,'o','MarkerSize',2);  xlabel('v_t');  ylabel('F_n'); 
p1 = polyfit(vt,Ft,2);  % Least square fitting
X = min(vt):.001:max(vt);  
y1 = polyval(p1,X); 
plot(X,y1); 

figure(4)
subplot(3,1,1)
plot(t,ForceTorque(:,1));
title('Force');
subplot(3,1,2)
plot(t,ForceTorque(:,2));
subplot(3,1,3)
plot(t,ForceTorque(:,3));


figure(5)
subplot(3,1,1)
plot(t,ForceTorque(:,4));
title('Torque')
subplot(3,1,2)
plot(t,ForceTorque(:,5));
subplot(3,1,3)
plot(t,ForceTorque(:,6));


figure(3)
subplot(3,1,1); hold on;
plot(t,footPosition(1,:));
title('Foot Position')
subplot(3,1,2); hold on;
plot(t,footPosition(2,:));
subplot(3,1,3); hold on;
plot(t,footPosition(3,:));


% figure()
% subplot(3,1,1);
% plot(t,footVelocity(1,:));
% title('Foot Velocity')
% 
% subplot(3,1,2)
% plot(t,footVelocity(2,:));
% 
% subplot(3,1,3)
% plot(t,footVelocity(3,:));


pause;



end










