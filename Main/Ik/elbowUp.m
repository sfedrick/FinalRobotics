function [theta1,theta2,theta3] = elbowUp(C, constants)
%ELBOWUP Summary of this function goes here
%   Detailed explanation goes here

    x0c = C(1);
    y0c = C(2);
    z0c = C(3);
    
    d1 = constants(1);
    a1 = d1;
    a2 = constants(2);
    a3 = constants(3);

%{
    theta1 = atan2(y0c,x0c);
    r1 = sqrt(y0c^2 + x0c^2);
    r2 = z0c - d1;
    phi2 = atan2(r2,r1);
    %r3 is just (x^2 + y^2 +z^2)^0.5 
    r3 = sqrt(r1^2 + r2^2);
    phi1 = acos((a3^2-a2^2-r3^2)/(-2*a2*r3));
    theta2 = phi2-phi1;
    phi3 = acos((r3^2-a2^2-a3^2)/(-2*a2*a3));
    theta3 = pi/2-phi3;
%}
  
   %{ 
    l2 = a2 + a1;
    r = sqrt(x0c^2 + y0c^2);
    theta1 = atan2( y0c,x0c);
    s = sqrt(r^2+z0c^2);
    theta3 = -pi/2 + acos((s^2-l2^2-a3^2)/(-2*l2*a3));
    theta3=-theta3;
    omega = pi/2 - theta3; 
    alpha = atan2((a3*sin(omega)),(l2+a3*cos(omega)));
    beta = atan2(z0c,r);
    theta2 = pi/2 - alpha - beta;
    %}
    r = sqrt(x0c^2 + y0c^2);
    
    
    z=z0c-d1;
    s = sqrt(r^2+(z)^2);
    
    theta1 = atan2( y0c,x0c);
    sigma=lawcosine(s,a2,a3);
    A2=lawcosine(a2,s,a3);
    A3=lawcosine(a3,a2,s);
    beta=atan2(z,r);
    theta2=pi/2-A3-beta;
    theta3=pi/2-sigma;
end

