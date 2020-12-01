function [theta1,theta2,theta3] = elbowUpTheta(C, constants,updown)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
    x0c = C(1);
    y0c = C(2);
    z0c = C(3);
    
     r = sqrt(x0c^2 + y0c^2);
     d1 = constants(1);
    a2 = constants(2);
    a3 = constants(3);
    
    z=z0c-d1;
    s = sqrt(r^2+(z)^2);
    
    theta1 = atan2( y0c,x0c);
    sigma=lawcosine(s,a2,a3);
    A2=lawcosine(a2,s,a3);
    A3=lawcosine(a3,a2,s);
    beta=atan2(z,r);
    theta2=pi/2-A3-beta;
    theta3=pi/2-sigma;
    
    theta1 = atan2( y0c,x0c)+updown*pi;
    theta2=-theta2;
    theta3=-pi-theta3;
end

