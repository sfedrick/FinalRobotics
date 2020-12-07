
%for shaun's machine
%globalParams(0.1,0.1,color,1);
function [] = globalParams(TMP,TVP,color,lynxpause)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
global toleranceMovementPause
toleranceMovementPause=TMP;

global toleranceVelocityPause
toleranceVelocityPause=TVP;

global Lynx
Lynx = ArmController(color);
pause(lynxpause);



end

