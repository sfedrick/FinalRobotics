function [time] = rosPause(timelimit)
%does a pause in rostime in seconds
    timelimit=timelimit*10^9;
    to=rostime("now");
    time=0;
    while(abs(time)<timelimit && time>=0)
        currentTime=rostime("now");
        time=to.Nsec()-currentTime.Nsec();
    end
end

