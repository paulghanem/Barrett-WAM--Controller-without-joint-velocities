
clear all 
load qd


fc = 180;
fs = 500;

[b,a] = butter(2,fc/(fs/2));
freqz(b,a)
for i=1:length(qd)
    

dataIn(i,:) = qd(i,:);
dataOut(i,:) = filter(b,a,dataIn(i,:));
end 
for j=1:7
figure
plot(dataOut(:,j))
figure
plot(dataIn(:,j))
end 