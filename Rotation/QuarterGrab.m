function [Stop] = QuarterGrab(q,qold,Error)
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
[~,RoboPose] = calculateFK(q);
[~,old] = calculateFK(qold);
RoboFingerZ=RoboPose(1:3,3);
StopAxisZ=old(1:3,3);
RoboFingerY=RoboPose(1:3,2);
StopAxisY=old(1:3,2);
StopAxisX=old(1:3,1);
RoboFingerX=RoboPose(1:3,1);
QuarterGrab2=StopAxisY'*RoboFingerY;
QuarterGrab1=StopAxisX'*RoboFingerX;
QuarterGrab3=StopAxisZ'*RoboFingerZ;
qdiff=norm(q-qold);
if(QuarterGrab2<(1-Error))
    Stop=true;
end


end

