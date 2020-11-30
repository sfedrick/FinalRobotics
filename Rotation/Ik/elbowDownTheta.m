function [theta1,theta2,theta3] = elbowDownTheta(C, constants,updown)
%ELBOWDOWN Summary of this function goes here
%   Detailed explanation goes here

    x0c = C(1);
    y0c = C(2);
    z0c = C(3);
    
    a1 = constants(1);
    a2 = constants(2);
    a3 = constants(3);

    theta1 = atan2(y0c,x0c)+pi*updown;
    z0c=z0c-a1;
    theta3=-pi/2 -acos((x0c^2+y0c^2+z0c^2-a2^2-a3^2)/(2*a2*a3));
    theta2=pi/2 -atan2(z0c,sqrt(x0c^2+y0c^2))+atan2(a3*sin(-pi/2-real(theta3)),(a2+a3*cos(-pi/2-real(theta3))));
    theta3=-(theta3+pi);
    theta2=-theta2;
end

