function [xc,yc,zc]=GetxC(R0e,P,d5)
xc = P(1) - d5*R0e(1,3);
yc = P(2) - d5*R0e(2,3);
zc = P(3) - d5*R0e(3,3);
end