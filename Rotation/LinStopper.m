function [StopMe,normdiff] = LinStopper(q,endpos,error,prevError)
StopMe=false;
[jointPositions,T0i] = calculateFK(q);
currentpos=jointPositions(6,:);
normdiff=norm(currentpos-endpos);
if(normdiff>(prevError+error))
    StopMe=true;
end
end

