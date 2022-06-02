% (x0,y0); r
% http://2000clicks.com/MathHelp/GeometryConicSectionCircleEquationGivenThreePoints.aspx
function [p0,r] = getCircle3Points(p)
    x1 = p(1,1); y1 = p(1,2);
    x2 = p(2,1); y2 = p(2,2);
    x3 = p(3,1); y3 = p(3,2);
    
    x0 = ((x1^2+y1^2)*(y3-y2) + (x2^2+y2^2)*(y1-y3) + (x3^2+y3^2)*(y2-y1)) / (2*(x1*(y3-y2)+x2*(y1-y3)+x3*(y2-y1)));
    y0 = ((x1^2+y1^2)*(x3-x2) + (x2^2+y2^2)*(x1-x3) + (x3^2+y3^2)*(x2-x1)) / (2*(y1*(x3-x2)+y2*(x1-x3)+y3*(x2-x1)));

    p0 = [x0 y0];
    r = sqrt((x1-x0)^2+(y1-y0)^2);
end
