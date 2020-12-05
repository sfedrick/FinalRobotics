function [qnew,isPos] = towardsbox(x,y,z,q)
%[0.75, 0.1, 0.7, -0.8, -pi/2,30]

%further from table
%[1.1, 0.1, 0.7, -0.8, -pi/2,30]
%closer to table
%q=[1.1000    0.1670    0.7030   -0.8680 -pi/2 30];

   [jointPositions,T0i] = calculateFK(q);
   T0i(1,4)=x;
   T0i(2,4)=y;
   T0i(3,4)=z;
    [qnew,isPos] = calculateIKs(T0i);
    qnew(5)=-pi/2;
    qnew(6)=q(6);
end