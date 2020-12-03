function [radius] = calculateRadiusForEndEff(lynx, color)
%CALCULATERADIUS 
% Calculates the radius r that the end effector needs to travel based on
% the given blocks. Finds the block with the greatest linear velocity
% coming towards the end effector and uses it to calculate r

% initialize
H=[];

[dynamicName, dynamicPose, dynamicTwist] = filterOutStaticBlocks(lynx);

% determine whether red or blue robot
if(strcmp(color,'red'))
    H=[1,0,0,200;
       0,1,0,200;
       0,0,1,0;
       0,0,0,1];
elseif(strcmp(color,'blue'))
   H=[-1,0,0,200;
       0,-1,0,200;
       0,0,1,0;
       0,0,0,1]; 
end

% transform blocks into end effector frame

% calc dot product of 

end

