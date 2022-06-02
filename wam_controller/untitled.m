

Q(1:4)=0; 
N(1:4)=0; 
S=[0,0];
A="up" ; 

for tk=1:1:1000
    simulate(S,A) 
end 




function A= manual(S)


for i=1:4 
    for j=1:4
    if Q(j) >Q(i)
        z=j;
    end 
    end 
end 

if z==1 
    A="up";
elseif z==2
    A="down";
elseif z==3
      A="left";
else A="right";
end 

[R,Sf]=simulate(S,A);


N(z)=N(z)+1;
Q(z)=Q(z)-1/N(z)*(R-Q(z));
   
S=Sf;



end 



function A= random(S)


z=unifrn(0,1); 

if z<=0.25 
    A="up";
elseif z>0.25 && z<=0.5
    A="down";
elseif z>0.5 && z <=0.75
      A="left";
else A="right";
end 

[R,Sf]=simulate(S,A);


N(z)=N(z)+1;
Q(z)=Q(z)-1/N(z)*(R-Q(z));
   
S=Sf;



end 





function [R,Sf]=simulate(S,A)


Wall=[ 0 5 ; 5 0 ; 5 3 ; 5 4 ; 5 5 ; 5 6 ; 5 7 ; 5 9 ;5 10 
       2 5 ; 3 5 ; 4 5 ; 6 4 ;7 4 ;9 4 ; 10 4 ] ; 
    


x=S(1);
y=S(2);

shuffle = unifrnd(0,1); 

switch A 
    case "up"
        
if (x==0 && y==0)
    if shuffle <0.9 
        x=x;
        y=y+1;
    else 
        x=x+1;
        y=y;
   
    end 
    
elseif (x==10 &&y==0)
        if shuffle<0.9
            x=x;
            y=y+1; 
        else
            x=x-1;
            y=y; 
        end 
elseif (x==0 && y==10) 
            if shuffle <0.9 
                x=x;
                y=y;
            else x=x+1;
                y=y; 
            end 
elseif (x==10 && y==10)
                if shuffle <0.9 
                    x=x;
                    y=y;
                else x=x-1;
                    y=y; 
                end 
elseif  y<10 && shuffle <= 0.8
    
    x=x;
    y=y+1; 
    
elseif  y<=10 && x <10 && 0.8<shuffle <= 0.9
    
    x=x+1;
    y=y;


elseif  y<=10 && x>0 && 0.9<shuffle <=1
    
    x=x-1;
    y=y;

end 



    case "down"
        
        
 if (x==0 && y==0)
    if shuffle <0.9 
        x=x;
        y=y;
    else 
        x=x+1;
        y=y;
   
    end 
    
elseif (x==10 &&y==0)
        if shuffle<0.9
            x=x;
            y=y; 
        else
            x=x-1;
            y=y; 
        end 
elseif (x==0 && y==10) 
            if shuffle <0.9 
                x=x;
                y=y-1;
            else x=x+1;
                y=y; 
            end 
elseif (x==10 && y==10)
                if shuffle <0.9 
                    x=x;
                    y=y-1;
                else x=x-1;
                    y=y; 
                end 
elseif  y>0 && shuffle <= 0.8
    
    x=x;
    y=y-1; 
    
elseif  y>=0 && x <10 && 0.8<shuffle <= 0.9
    
    x=x+1;
    y=y;


elseif  y>=10 && x>0 && 0.9<shuffle <=1
    
    x=x-1;
    y=y;

end      
        
        
        
        
    

    case "left" 
        
    if (x==0 && y==0)
    if shuffle <0.9 
        x=x;
        y=y;
    else 
        x=x;
        y=y+1;
   
    end 
    
elseif (x==10 &&y==0)
        if shuffle<0.9
            x=x-1;
            y=y; 
        else
            x=x;
            y=y+1; 
        end 
elseif (x==0 && y==10) 
            if shuffle <0.9 
                x=x;
                y=y;
            else x=x;
                y=y-1; 
            end 
elseif (x==10 && y==10)
                if shuffle <0.9 
                    x=x-1;
                    y=y;
                else x=x;
                    y=y-1; 
                end 
        
        
        
    elseif  x>0 && shuffle <= 0.8
    
    x=x-1;
    y=y;
 
    
    elseif  x>=0 && y <10 && 0.8<shuffle <= 0.9
    
    x=x;
    y=y+1;

   

    elseif  x>=0 && y>0 && 0.9<shuffle <=1
    
    x=x;
    y=y-1;

end


    case "right" 
        
        if (x==0 && y==0)
    if shuffle <0.9 
        x=x+1;
        y=y;
    else 
        x=x;
        y=y+1;
   
    end 
    
elseif (x==10 &&y==0)
        if shuffle<0.9
            x=x;
            y=y; 
        else
            x=x;
            y=y+1; 
        end 
elseif (x==0 && y==10) 
            if shuffle <0.9 
                x=x+1;
                y=y;
            else x=x;
                y=y-1; 
            end 
elseif (x==10 && y==10)
                if shuffle <0.9 
                    x=x;
                    y=y;
                else x=x;
                    y=y-1; 
                end 
        
elseif  x<10 && shuffle <= 0.8
    
    x=x+1;
    y=y;
 
    
elseif  x <=10 && y <10 && 0.8<shuffle <= 0.9
    
    x=x;
    y=y+1;


elseif  x<=10 && y>0 && 0.9<shuffle <=1
    
    x=x;
    y=y-1;

end
 
end 

if x==10 && y==10 
    R=R+1; 
end 



Sf(1)=x;
Sf(2)=y; 


for k= 1: length(Wall) 
    
    if Sf==Wall(k)
        Sf=S;
        break;
end
end

end 


