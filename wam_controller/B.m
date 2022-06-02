function B=B(q,qd)
C=Cjk(q);
for i=1:7
    for j=1:7
        for k=1:7
       
       H(k)= C(i,j,k)*qd(k);     
            
            
        end 
        B(i,j)=sum(H);
        
    end 
    
end 
end 