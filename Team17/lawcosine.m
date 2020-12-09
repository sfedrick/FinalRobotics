function [A] = lawcosine(a,b,c)
A=((a^2)-b^2-c^2)/(-2*b*c);
A=acos(A);
end

