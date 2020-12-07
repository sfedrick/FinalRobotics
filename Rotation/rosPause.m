
function [currentTime] = rosPause(timelimit)
%does a pause in rostime in seconds

    to=RossyTime();
    currentTime=RossyTime()-to;
    while(currentTime<timelimit)
        currentTime=RossyTime()-to;
        pause(0.01);
    end
end

