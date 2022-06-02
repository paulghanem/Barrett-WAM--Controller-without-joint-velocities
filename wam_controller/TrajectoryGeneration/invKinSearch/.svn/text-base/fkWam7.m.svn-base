function A07 = fkWam7(q)
% Put origin of Frame 7 at intersection of wrist axes.
%syms a3 d3 d5 
a3 = 0.045;  d3 = 0.55;  d5 = 0.3;   d7 = 0.0161;
A01 = trotz(q(1))                   * round(trotx(-pi/2));
A12 = trotz(q(2))                   * round(trotx(pi/2)) ;
A23 = trotz(q(3)) * transl(a3,0,d3) * round(trotx(-pi/2));
A34 = trotz(q(4)) * transl(-a3,0,0) * round(trotx(pi/2)) ;
A45 = trotz(q(5)) * transl(0,0,d5)  * round(trotx(-pi/2));
A56 = trotz(q(6))                   * round(trotx(pi/2)) ;
A67 = trotz(q(7)) * transl(0,0,d7)                       ;
A07 = A01 * A12 * A23 * A34 * A45 * A56 * A67;