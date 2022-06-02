function [time TRAJ vel] = orTrajPlanningHelper(targetTransform, problemHandle)

d7 = 0.154;% meters
targetTransform = targetTransform/transl(0, 0, d7);

%place the first TSR's reference frame at the object's frame relative to world frame
T0_w = targetTransform;

%get the TSR's offset frame in w coordinates
Tw_e1 = MakeTransform(rodrigues([0 0 0]),[0 0 0]');

%define bounds to only allow rotation of the hand about z axis and a small deviation in translation along the z axis
epsilon = 0.00001;
Bw = [-1 1 -1 1 -1 1  -1 1 -1 1 -1 1]*epsilon;

%Bw = [0 0   0 0   -0.02 0.02   0 0   0 0   -.3 .3];

%serialize the strings
TSRstring1 = SerializeTSR(0,'NULL',T0_w,Tw_e1,Bw);
TSRChainString = SerializeTSRChain(0,1,0,1,TSRstring1,'NULL',[]);

orProblemSendCommand(['RunCBiRRT psample 0.25 ' TSRChainString], problemHandle);

if exist('cmovetraj.txt', 'file')
    [time TRAJ vel] = readTraj();
else
    error('Trajectory planning failed!');
end

end