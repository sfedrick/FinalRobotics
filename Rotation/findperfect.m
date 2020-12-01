%find strike positions 
function [q,isPos] = findperfect(r)
%[0.75, 0.1, 0.7, -0.8, -pi/2,30]
%[1.1, 0.1, 0.7, -0.8, -pi/2,30]
%closer to table
%[1.1000    0.1670    0.7030   -0.8680 -pi/2 30]
   [jointPositions,T0i] = calculateFK([1.1, 0.1, 0.7, -0.8, -pi/2-0.1,30]);
    endpos=[200,200,T0i(3,4)];
    pos=[T0i(1,4),T0i(2,4),T0i(3,4)];
    posdif=endpos-pos;
    posdif=posdif/norm(posdif);
    T0i(1,4)=T0i(1,4)+r*posdif(1);
    T0i(2,4)=T0i(2,4)+r*posdif(2);
    T0i(3,4)=T0i(3,4)-10;
    [q,isPos] = calculateIK(T0i);
    q(5)=-pi/2;
    q(6)=30;
end