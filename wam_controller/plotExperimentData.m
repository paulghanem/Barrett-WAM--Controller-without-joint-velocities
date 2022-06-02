function plotExperimentData( experiment )
%PLOTEXPERIMENTDATA Summary of this function goes here
%   Detailed explanation goes here

t = experiment.frameTimes;
wamData = experiment.wamData;

% Go through each joint.  Make a new figure for each joint
for i = 1:7
   figure(i);
   subplot(3,1,1);
   hold on;
   plot(t,wamData.q(:,i),'r');
   plot(t,wamData.qdes(:,i),'b');
   legend('q','qdes');
   hold off;
   
   subplot(3,1,2);
   hold on;
   plot(t,wamData.qd(:,i),'r');
   plot(t,wamData.qddes(:,i),'b');
   legend('qd','qddes');
   hold off;
   
   subplot(3,1,3);
   hold on;
   plot(t,wamData.tau(:,i),'r');
   legend('tau');
   hold off;
end

end

