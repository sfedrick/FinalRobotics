function [T0e, targetBlockName] = calculateStrikePosForTargetBlock(dynBlockNames, dynBlockPoses, color)
%CALCULATESTRIKEPOSFORTARGETBLOCK 
% Calculates the x,y coordinates of the striking position for the end
% effector. 

% Radial vector in world frame. A point on this line is calculated based on
% the radiusFromCenter, which is then used to set the XY coordinate of the
% striking pose in the robot's frame
if (strcmp(color,'blue'))
    radialVector = [0 1];
else
    radialVector = [0 -1];
end

% Height that we want robot
strikePoseHeight = 80;

% Find target block
[targetBlockIdx, targetBlockName] = calculateTargetBlock(dynBlockNames, dynBlockPoses, color);

% Only proceed if target block idx is not nan. If it is, then that means
% there is no block in our target region of the turnstile
if (~isnan(targetBlockIdx))
    
    % Get the target block XY coords
    targetBlockXYCoord = dynBlockPoses{targetBlockIdx}(1:2,4);
    
    % Calculate radius from center of turnstile to the target block
    radiusFromCenter = norm(targetBlockXYCoord);
    
    % Calculates the XY position we want the end effector to be in in the
    % world frame
    endEffXYPosInWorldFrame = radialVector*radiusFromCenter;
    
    % Creates the T matrix for the end effector in the world frame
    if (strcmp('blue',color))
        endEff_T_worldFrame = [1 0 0 endEffXYPosInWorldFrame(1);
                                0 1 0 endEffXYPosInWorldFrame(2);
                                0 0 -1 strikePoseHeight;
                                0 0 0 1];
       Hw0=[-1,0,0,200;
           0,-1,0,200;
           0,0,1,0;
           0,0,0,1]; 
    else
        endEff_T_worldFrame = [-1 0 0 endEffXYPosInWorldFrame(1);
                                 0 -1 0 endEffXYPosInWorldFrame(2);
                                 0 0 -1 strikePoseHeight;
                                 0 0 0 1];
       Hw0=[1,0,0,200;
           0,1,0,200;
           0,0,1,0;
           0,0,0,1]; 
    end    
    
    % Transform end effector T in world frame to robot frame
    endEff_T_robotFrame = Hw0*endEff_T_worldFrame;
    T0e = endEff_T_robotFrame
    return;
end

% If there is no target block, then output NaN
T0e = NaN;

end

