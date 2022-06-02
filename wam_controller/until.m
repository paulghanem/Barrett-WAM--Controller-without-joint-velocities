function q = until
x(:,1) = statespace2(0,0,0);
q(1,:) = transpose(x(1));
qd(1,:) = transpose(x(2));
for i=2:523,
    x(:,i)= statespace2(transpose(q(i-1,:)),transpose(qd(i-1,:)),[4;0;0;0;0;0;0]);
    q(i,:)= transpose(x((1),i));
    
    qd(i,:)= transpose(x((2),i));
    
     
end 
