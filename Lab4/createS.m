function [S] = createS(a)
% Creates skew symmetric matrix
ax=a(1);
ay=a(2);
az=a(3);
S=[0,-az,ay;
   az,0,-ax;
   -ay,ax,0];

end

