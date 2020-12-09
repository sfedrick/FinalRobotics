function [currentTime] = rosPause(timelimit)
%does a pause in rostime in seconds

    to=RossyTime();
    currentTime=RossyTime()-to;
    while(currentTime<timelimit)
        currentTime=RossyTime()-to;
        i=0;
        while i<100000
            i=i+1;
        end
    end
end

