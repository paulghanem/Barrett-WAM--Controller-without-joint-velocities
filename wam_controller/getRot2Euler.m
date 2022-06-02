% from Craig, pp. 45-49
function ypr = getRot2Euler(R)
    % ypr = [yaw pitch roll], there are 2 solutions so 2 rows
    ypr = nan(2,3);
    
    % compute beta (pitch)
    ypr(:,2) = [atan2(-R(3,1), sqrt(R(1,1)^2+R(2,1)^2)); atan2(-R(3,1), -sqrt(R(1,1)^2+R(2,1)^2))];
    
    % compute alpha (yaw)
    ypr(:,1) = atan2([R(2,1); R(2,1)]./cos(ypr(:,2)), [R(1,1); R(1,1)]./cos(ypr(:,2)));
    
    % compute gamma (roll)
    ypr(:,3) = atan2([R(3,2); R(3,2)]./cos(ypr(:,2)), [R(3,3); R(3,3)]./cos(ypr(:,2)));
    
    
    % we will always use the answer with the positive sqrt
    % NOTE: whenever pitch is +/- pi/2, there is an infinite number of solutions
    ypr = ypr(1,:);
end
