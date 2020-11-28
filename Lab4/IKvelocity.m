% This function calculates the angular joint velocities given:
% v - vector of linear joint velocities
% qconfig - vector of angular joint positions
% joint - joint of interest in robot arm

function [qDesired] = IKvelocity(v,qconfig,joint)

J=CreateJac(qconfig,joint);
a=0;

%removes NaN rows from J and V because they do not matter
for i=1:length(v)
    a=a+1;
    if(isnan(v(a)))
        v(a)=[];
        J(a,:)=[];
        a=a-1;
    end
end

% Calc qDesired. If v is transposed for some reason, then we want to flip
% it
[row,col]=size(v);
if(col==1)
    qDesired=J\v;
else
    v=v';
    qDesired=J\v;
end

% Check the rank. If the rank is decreased, then this means that Jacobian
% has lose rank and is therefore an infeasible configuration
Rj=rank(J);
Rv=rank([J,v]);
if(Rj==Rv)
    disp("this is feasible");
else
    disp("this is infeasible the given dq is a least squared approximation");
end

end

