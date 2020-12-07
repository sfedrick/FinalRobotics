function [time] = RossyTime()
%Because ross is fucken stupid we have to  amke a different function for
%getting rostime
t=rostime("now");
%  covert rostime to a double
time=double(t.Sec)+double(t.Nsec)*10^-9;
end

