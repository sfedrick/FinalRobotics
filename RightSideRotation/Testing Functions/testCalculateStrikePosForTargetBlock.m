function [] = testCalculateStrikePosForTargetBlock(testNum)
%TESTCALCUALTESTRIKEPOSFORTARGETBLOCK 
% Tests calculateStrikePosForTargetBlock
if (testNum == 1)
    color = 'blue';
    name = {'dynamic_block_4'};
    pose = {[  0.9320    0.0000    0.3625   83.4990;
            -0.3625    0.0000    0.9320   36.3043;
            0.0000   -1.0000    0.0000   59.9998;
             0         0         0    1.0000]};

    [T0e] = calculateStrikePosForTargetBlock(name, pose, color);

    [q isPos] = calculateIKs(T0e);

    plotJointPos(q,'b',5);
end

if (testNum == 2)
    global lynx;
    color = 'blue'
    lynx = ArmController(color);
    pause(1);
    
    [dynName, dynPose, dynColor] = filterOutStaticBlocks();
    
    [T0e] = calculateStrikePosForTargetBlock(dynName, dynPose, color);

    if (isnan(T0e))
        disp('Failed');
        return;
    end
    [q isPos] = calculateIKs(T0e);
    
    move([q, 30],lynx);
end

end

