function [r] = calculateRadiusForEndEff(lynx, color,axis,directionlimit)
%CALCULATERADIUS 
% Calculates the radius r that the end effector needs to travel based on
% the given blocks. Finds the block with the greatest linear velocity
% coming towards the end effector and uses it to calculate r

% this is the world y axis that we want to align with most
 safety=-15;
%JerkMove(lynx,start,0.1,2,0.1,5)
worldYaxis=axis;

% initialize values

maxDotVal = directionlimit;
maxBlockVelName = '';
maxBlockRadius = 0;
Hw0=[];

% tolerance value for linear velocity comparison
linVelTolerance = 0.1;

% set Hw0 depending on color of the robot
if(strcmp(color,'red'))
    Hw0=[1,0,0,-200;
       0,1,0,-200;
       0,0,1,0;
       0,0,0,1];
elseif(strcmp(color,'blue'))
   Hw0=[-1,0,0,200;
       0,-1,0,200;
       0,0,1,0;
       0,0,0,1]; 
end

[dynamicName, dynamicPose, dynamicTwist] = filterOutStaticBlocks(lynx);

for i=1:length(dynamicName)
    % get each block's x & y linear velocities in the world's frame
    blockVel = dynamicTwist(i);
    blockVel = blockVel{1};
    blockVel = blockVel(1:2);
    
    % get each block's x & y positions w.r.t world frame
    blockPos = dynamicPose(i);
    blockPos = blockPos{1};
    blockPos = blockPos(1:2,4);
    
    % transform each block into end effector frame
    normalizedBlockVel = blockVel/norm(blockVel);
    
    % dot each block's velocity with worldYaxis and get the maximum
    newMaxDotVal = worldYaxis*normalizedBlockVel;
    
    % get the radius of the current block w/ relation to the world frame,
    % which is just the norm of current block position
    newMaxBlockRadius = norm(blockPos);
    
    % if dot prod of velocity vector and radius of block w.r.t world frame
    % is larger than max, then set new max
    if ((newMaxDotVal - linVelTolerance) >= maxDotVal && newMaxBlockRadius >= maxBlockRadius)
        maxDotVal = newMaxDotVal;
        maxBlockRadius = newMaxBlockRadius;
        maxBlockVelName = dynamicName(i);
    end
    
end

% once we have the name of the max block velocity, we calculate the
% distance (radius) that we want the end effector to travel in the radial
% direction

% get the current end effector position
[jointPos, T0e] = calculateFK(lynx.get_state());
p0e = [jointPos(6,:)';1];

% transform the end effector position from robot frame to world frame
pwe = Hw0*p0e;
pwe(3)=0;
pwe(4)=0;
% then take the norm of it b/c we only want the distance
normPwe = norm(pwe);

% get the radial distance r that we want the robot to travel
%r = norm(pwe) - maxBlockRadius;
% r=r+safety;
r=90-maxBlockRadius;
if(r<0)
    r=5;
end
%disp('Desired block')

if(maxDotVal<=directionlimit)
    r=nan;
end
end

