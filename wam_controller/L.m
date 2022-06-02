
function C4=L(q,qdd)
C0=C1(q);
for i=1:7
    for j=1:7
        for k=1:7
            C(k)=C0(i,j,k)*qdd(k);
        end 
        C3(i,j)=sum(C);
    end 
end 


    C4=C3+lingrav(q);
    
end     