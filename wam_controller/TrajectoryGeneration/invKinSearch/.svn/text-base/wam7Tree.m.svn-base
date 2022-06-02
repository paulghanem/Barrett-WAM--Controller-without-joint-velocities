function [q] = wam7Tree(qset, length)
    
for j = 1:size(qset(length).soln,2)
    qset(length).path(j).set=j;%path vector
    qset(length).path(j).val=0;%sum of lower pentalty functions
end
    for i = (length-1):-1:1
        current_q = qset(i).soln;
        q_below = qset(i+1).soln;
        val=[];
        for k = 1:size(q_below,2)
            val(k)= qset(i+1).path(k).val;
        end
      
        for j = 1:size(current_q,2)
            
        comparator = current_q(:,j)*ones(1,size(q_below,2)) - q_below;
        [minv, minLoc] = min(cNorm(comparator)+val);
        qset(i).path(j).set = [j , qset(i+1).path(minLoc).set];
        qset(i).path(j).val = minv;
        
        end
     
    end
    val=[];
    for k = 1:size(qset(1).path,2)
            val(k)= qset(1).path(k).val;
    end
    [~,minLoc]=min(val);
    q= qset(1).path(minLoc).set;
    
end

function [twoNorm] = cNorm(M) %computes the norm of each column in a matrix
        twoNorm = sqrt(sum(abs(M).^2,1)); 
end

function [res] = penalty(nor) %function of how 
res = nor;
end