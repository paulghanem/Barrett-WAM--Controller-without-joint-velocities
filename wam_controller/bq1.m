function bq = bq1(qd)



s=20;
bc = [2.516; 2.581; 2.038; 0.956];
B =  [1.142; 0.946; 0.309; 0.255];

bq = bc.*atan(s*qd)*2/pi+B.*qd;
end 