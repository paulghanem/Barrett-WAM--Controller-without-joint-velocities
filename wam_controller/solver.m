clear all

global mr a1 a2 L1 L2 a3 a4
mr=[3.643E-03;3.561E-03;3.243E-03;2.806E-03];
L1=0;
L2=110;
a1=0;
a2=100;
x0=[190;270];
[x,fval]=fsolve(@calcs,x0,optimset('MaxFunEvals', 90000, 'MaxIter', 10, 'FunValCheck', 'on'));
a3=x(1)
a4=x(2)
x0=[0.0150;0.1710]
[x,fval]=fsolve(@calcs2,x0,optimset('MaxFunEvals', 90000, 'MaxIter', 10, 'FunValCheck', 'on'));
L3=x(1);
L4=x(2);
function F=calcs(x)
global mr a1 a2
F=[(mr(1)*cos(a1))+(mr(2)*cos(a2))+(mr(3)*cos(x(1)))+(mr(4)*cos(x(2)));
    (mr(1)*sin(a1))+(mr(2)*sin(a2))+(mr(3)*sin(x(1)))+(mr(4)*sin(x(2)))];
end

function F=calcs2(x)
global mr a1 a2 L1 L2 a3 a4
F=[((mr(2)*cos(a2))*L2)+((mr(3)*cos(a3))*x(1))+((mr(4)*cos(a4))*x(2));
((mr(2)*sin(a2))*L2)+((mr(3)*sin(a3))*x(1))+((mr(4)*sin(a4))*x(2))];
end
