<<<<<<< HEAD
=======

>>>>>>> 7a881bd25aec58985899ed8378c6b9ab3c80731c
function [currentTime] = rosPause(timelimit)
%does a pause in rostime in seconds

    to=RossyTime();
    currentTime=RossyTime()-to;
    while(currentTime<timelimit)
        currentTime=RossyTime()-to;
        pause(0.01);
    end
end

