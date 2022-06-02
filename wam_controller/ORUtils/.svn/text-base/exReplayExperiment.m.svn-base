function exReplayExperiment( experimentFile, speedup )
%EXREPLAYEXPERIMENT Replays an experiment straight from the mat file retrieved from readWAMData
%   Detailed explanation goes here

load(experimentFile);

if nargin < 2
   speedup = 1; 
end

I = speedup*10;

L = ceil(size(Q,1)/I);

A = Q(1:I:end,:)';
B = zeros(1,L);
%C = GripperPosition(1:I:end,:)';
C = zeros(1,L);

TRAJ = [A ; B ; C];
exDisplayTrajectory(1,TRAJ);

%TRAJ = [Q(1:end,:)'; zeros(1,size(Q,1)); GripperPosition(1:end,:)'];
%orRobotStartActiveTrajectory(1,TRAJ,t*10);

end

