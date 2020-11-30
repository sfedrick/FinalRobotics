function [discreteLine] = makeLine(initial,final,step)

% Creates a discretized line from an initial point to a final point with
% the number of steps in between
t=0:1/(step-1):1;
v=final-initial;

discreteLine=[];
    for i=1:length(t)
    newD=initial +t(i)*v;
    discreteLine=[discreteLine;newD];
    end 
end

