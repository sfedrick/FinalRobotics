function [] = testCalculateTargetBlock()
% Tests calculateTargetBlock

color = 'red';
global lynx;
lynx = ArmController(color);
pause(1);

[dynamicName, dynamicPose, dynamicTwist] = filterOutStaticBlocks();

[targetBlockName] = calculateTargetBlock(dynamicName, dynamicPose, color)

end

