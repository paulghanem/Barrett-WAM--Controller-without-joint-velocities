
%initializeWAMController();
%wamInfo = initWamInfo();

%load Option1_02-13-2012_1750.mat;
%plotWAMData(t,Q,Qdes,Qd,Qddes,wamInfo,20.5,31.8,1,0);

%load Option2_02-13-2012_1755.mat;
%load Option2_02-14-2012_1607.mat;
%plotWAMData(t,Q,Qdes,Qd,Qddes,wamInfo,20.5-2.188,30.95-2.188,2,2.188);

%load Option3.mat;
%plotWAMData(t,Q,Qdes,Qd,Qddes,Qdd,Qdddes,Force,Torque,wamInfo,.002,length(t)/500);
%plotWAMData(t,Q,Qdes,Qd,Qddes,Qdd,Qdddes,Force,Torque,wamInfo,.002,length(t)/500,0,0,0);
%plotWAMData(t,tdes,Q,Qdes,Qd,Qddes,Qdd,Qdddes,QFilter,QdFilter,QddFilter,Force,Torque,ForceOriginalFilter,TorqueOriginalFilter,ForceNewFilter,TorqueNewFilter,GripperForce,Type,wamInfo,.002,length(t)/500-1,0,0,0,1);
%plotWAMData(t,     Q,Qdes,Qd,Qddes,Qdd,Qdddes,                           Force,Torque,ForceOriginalFilter,TorqueOriginalFilter,ForceNewFilter,TorqueNewFilter,GripperForce,SensorExpFT,Type,wamInfo,.002,length(t)/500-1,0,0,0,1)
plotWAMData('testNew.mat',wamInfo,.002,100,1,0,0,0);

