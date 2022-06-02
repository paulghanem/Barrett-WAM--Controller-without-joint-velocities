t_d2=q.time;
t_d3=x2.time;
t_d4=tau.time;
e=e1.signals.values;
position=q.signals.values;
velocity=x2.signals.values;
torques=tau.signals.values;


plot(t_d2,position(:,1),t_d2,x1(1:length(t_d2),1));
for j = 1:4
    
    subplot(4,2,j),plot(t_d2,(position(:,j))*180/pi,t_d2,(x1(1:length(t_d2),j)-e(:,j))*180/pi);
    title(['Position: Joint',num2str(j)])
    
end 
figure
for j = 1:4
    
   
    subplot(4,2,j),plot(t_d3,velocity(:,j)*180/pi,t_d3,xd1(1:length(t_d3),j)*180/pi);
    title(['velocity: Joint',num2str(j)])
end 
figure

for j = 1:4
    
   
    subplot(4,2,j),plot(t_d4,torques(:,j));
    title(['torques: Joint',num2str(j)])
end 
