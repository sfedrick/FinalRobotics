function [successfullyGrabbed] = grabBlock(endEffStrikeLine, desiredBlockName, endEffStrikePose, color)
%GRABBLOCK 
% Given a desired rotating block, grab it

global lynx;

[endEffStrikeConfig isPos] = calculateIKs(endEffStrikePose);

% first move to the end effector strike pose
move([endEffStrikeConfig, 30],lynx);

% this is the angle from the end effector's strike "line" that we want the
% block to be at before we tell the end effector to go down and grab the
% block. initialize to Inf
angleFromEndEff = Inf;

% the index of the desired block out of all the blocks. need to do this b/c
% we don't want to filter out the dynamic blocks each time we poll
[name, pose, twist] = lynx.get_object_state();
desiredBlockIdx = getIndexOfBlockForName(name, desiredBlockName);

% once here, wait until the block is 30 degrees from the robot
while(angleFromEndEff > 30)
    
    % keep polling the block's position
    [name, pose, twist] = lynx.get_object_state();
    
    % get the desired block's current XY coord vector
    desiredBlockXYCoord = pose{desiredBlockIdx}(1:2,4);
    
    % get current angle between desired block and end effector strike line
    angleFromEndEff = calcSmallerAngleBwTwoVectors(endEffStrikeLine,desiredBlockXYCoord);
    pause(0.2);
end

% once block is 30 degrees from robot, then move down by 20 in height
endEffStrikePose(3,4) = endEffStrikePose(3,4) - 20;

% calculate the new lowered config q
[endEffLoweredConfig isPos] = calculateIKs(endEffStrikePose);

disp(endEffLoweredConfig);

% move to the lowered config
lynx.set_pos([endEffLoweredConfig, 30]);
disp('lowering');
pause(1);

% grip the block
lynx.set_pos([endEffLoweredConfig, -15]);
disp('closing');

% check to see if we successfully grabbed the block
[pos, vel] = lynx.get_state();
if (pos(6) > 0)
    successfullyGrabbed = true;
else
    successfullyGrabbed = false;
end

end

