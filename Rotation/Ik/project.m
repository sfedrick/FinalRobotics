function [U] = project(normal,Uin)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
projUn=(dot(normal,Uin))/(norm(normal)^2);
Un=projUn*normal;
U=Uin-Un;
end

