function [time pos vel] = readTraj()


S = textread('cmovetraj.txt','%s');


indexOffset = 37;
outindex=1;

while 1
   if isempty(str2num(S{indexOffset}))
       return;
   end
    
    time(outindex)=str2double(S{indexOffset+14});
    for a = 1:7
        index = indexOffset + a -1; 
        pos(a,outindex) = str2double(S{index});
        
        
    end
    for a = 1:7
        index = indexOffset + a + 7 - 1; 
        vel(a,outindex) = str2double(S{index});
        
    end
    
    indexOffset = indexOffset +15;
    outindex = outindex+1;
end

end