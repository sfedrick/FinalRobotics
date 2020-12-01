function [Stop] = QuarterGrab(lynx,qold,Error)
% This function alerts you when the robot has a completed a quater rotation
% around the table
%lynx is the lynx variable from lynx=ArmController(color)
%color is a character string
%Range is the detection radius of the robot a typical value is 100
%Direction is a number from -1 to 1 that determines the acceptable direction 
%normally direction >0 means boxes that are in front of robot
%direction<0 are boxes that are behind the robot end effector a typical value 
% is Direction=.70;
%Axis determines the axis of the end effector you are concerned with 
%1= x axis 2=y axis 3=zaxis you typically only need 1 or 3 
%stopaxis is the axis you want to get close to
%stop is a boolean
Stop=false;

[q,~]=lynx.get_state();

[~,RoboPose] = calculateFK(q);
[~,old] = calculateFK(qold);
RoboFinger=RoboPose(1:3,3);
StopAxis=old(1:3,3);
QuarterGrab=StopAxis'*RoboFinger;
if(QuarterGrab<(1-Error))
    Stop=true;
end


end
