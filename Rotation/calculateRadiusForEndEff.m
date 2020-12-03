function [radius] = calculateRadiusForEndEff(lynx, color)
%CALCULATERADIUS 
% Calculates the radius r that the end effector needs to travel based on
% the given blocks. Finds the block with the greatest linear velocity
% coming towards the end effector and uses it to calculate r

% initialize transf of world frame to robot base frame
Hw0=[];

[dynamicName, dynamicPose, dynamicTwist] = filterOutStaticBlocks(lynx);

% determine whether red or blue robot
if(strcmp(color,'red'))
    Hw0=[1,0,0,200;
       0,1,0,200;
       0,0,1,0;
       0,0,0,1];
elseif(strcmp(color,'blue'))
   Hw0=[-1,0,0,200;
       0,-1,0,200;
       0,0,1,0;
       0,0,0,1]; 
end

[jointpos, T0e] = calculateFK(lynx.get_state());

for i=1:length(dynamicName)
    % get each block's linear velocity
    blockVel = dynamicTwist(1:3);
    
    % transform each block into end effector frame
    currBlockDirection = Hw0*T0e;
    normCurrBlockDirection = currBlockDirection/norm(currBlockDirection);
    
    % calculate resultant linear velocity vector for each block and dot it with
    % lynx's z
    
end

% calc dot product of 

end

