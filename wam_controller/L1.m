function C4=L1(q,qdd)
C0=C1_1(q);
for i=1:4
    for j=1:4
        for k=1:4
            C(k)=C0(i,j,k)*qdd(k);
        end 
        C3(i,j)=sum(C);
    end 
end 


    C4=C3+lingrav1(q);
    
end     