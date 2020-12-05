function [] = pickRightSideBlocks(color)
%PICKRIGHTSIDEBLOCKS 

% Selects blocks on the 6 o'clock - 3 o'clock of the turn table (from 
% perspective of blue robot). Right now it waits until the target block
% gets to 3 o'clock before grabbing it. However, the "time" position will
% hopefully be able to be moved to between 12 o'clock and 6 o'clock. 

% For best results at current state, try to run it when a block is nearing the 6
% o'clock - 5 o'clock mark.

% Also only works for blue b/c haven't added logic for red yet, but will do
% that later

global lynx;
lynx = ArmController('blue');
pause(1);

% Filter out the static blocks first
[dynamicName, dynamicPose, dynamicTwist] = filterOutStaticBlocks();

% Calculate the strike position to hover over the target block
[T0e, targetBlockName] = calculateStrikePosForTargetBlock(dynamicName, dynamicPose, color);

% Grab the block
[successfullyGrabbed] = grabBlock([0 1], targetBlockName, T0e, color);

if (successfullyGrabbed)
    lynx.set_pos([0 0 0 0 0 -15]);
end

end

