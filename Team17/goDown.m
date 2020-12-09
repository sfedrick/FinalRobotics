function [q,isPos] = goDown(q,h)
%[0.75, 0.1, 0.7, -0.8, -pi/2,30]

%further from table
%[1.1, 0.1, 0.7, -0.8, -pi/2,30]
%closer to table
%[1.1000    0.1670    0.7030   -0.8680 -pi/2 30]

   [jointPositions,T0i] = calculateFK(q);
    T0i(3,4)=h;
    [q,isPos] = calculateIK(T0i);
    %q(5)=-pi/2;
    q(6)=30;
end