function [targetBlockIdx, targetBlockName] = calculateTargetBlock(dynBlockNames, dynBlockPoses, color)
%CALCULATETARGETBLOCK 
% Finds the block closest to the 3 o'clock mark of the turnstile from the
% blue robot's perspective. Returns the idx of target block in the list of
% dynamic blocks, NOT list of static blocks

minAngle = Inf;
targetBlockName = '';
targetBlockIdx = NaN;

% we set the quadrant's cutoff line at vector of 3 o'clock, so in the
% world frame this is a unit vector of (0,1) or (0,-1) if red robot
if (strcmp(color,'blue'))
    quadrantLimit = [0 1];
else
    quadrantLimit = [0 -1];
end


for i=1:length(dynBlockNames)
    
     % save off current block name
    currBlockName = dynBlockNames{i};
    disp(currBlockName)
    
    % get the block's X and Y coordinates only
    blockXYCoords = dynBlockPoses{i}(1:2,4);
    disp(blockXYCoords);
    
    % if we're the blue robot, then only look at blocks w/ +x and +y pose
    % values
    % but if red robot, then only look at blocks w/ -x and -y pose values
    if ((strcmp(color,'blue') && blockXYCoords(1) > 0 && blockXYCoords(2) > 0) || (strcmp(color,'red') && blockXYCoords(1) < 0 && blockXYCoords(2) < 0))
        
        % gets the angle from the block position vector to the robot's
        % strike line of 3 o'clock
        newMinAngle = calcSmallerAngleBwTwoVectors(quadrantLimit, blockXYCoords);
        disp(newMinAngle)
        
        % if this is the smallest angle so far, then this is the block we
        % want
        if (newMinAngle < minAngle)
            minAngle = newMinAngle;
            targetBlockName = currBlockName;
            targetBlockIdx = i;
        end
        
    end
end
disp('Target block')
disp(targetBlockName)

end

