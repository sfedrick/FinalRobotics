function [targetBlockIdx] = calculateTargetBlock(dynBlockNames, dynBlockPoses, color)
%CALCULATETARGETBLOCK 
% Finds the block closest to the right hand side of the bottom-left
% quadrant of the rotating table by angle

minAngle = Inf;
targetBlockName = '';
targetBlockIdx = NaN;

% we set the quadrant's cutoff line at vector = (0,1), so in the
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
    
    % Only look at blocks located in the bottom left quadrant, which means
    % +x and +y values
    blockXYCoords = dynBlockPoses{i}(1:2,4);
    disp(blockXYCoords);
    
    % if we're the blue robot, then only look at blocks w/ +x and +y pose
    % values
    % but if red robot, then only look at blocks w/ -x and -y pose values
    if ((strcmp(color,'blue') && blockXYCoords(1) > 0 && blockXYCoords(2) > 0) || (strcmp(color,'red') && blockXYCoords(1) < 0 && blockXYCoords(2) < 0))
        
        
        % calculate angle between block and cutoff vector in degrees
        newMinAngle = acos((quadrantLimit*blockXYCoords)/(norm(blockXYCoords)*norm(quadrantLimit)))*180/pi;
        
        % want the smaller of the two angles, so subtract 90 if greater
        % than 90 b/c acos has 2 solns
        if (newMinAngle > 90)
            newMinAngle = newMinAngle - 90;
        end
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

