function [T0e, targetBlockName] = calculateStrikePosForTargetBlock(dynBlockNames, dynBlockPoses, color)
%CALCULATESTRIKEPOSFORTARGETBLOCK 
% Calculates the x,y coordinates of the striking position for the end
% effector. 

% Radial vector in world frame. A point on this line is calculated based on
% the radiusFromCenter, which is then used to set the XY coordinate of the
% striking pose in the robot's frame
radialVector = [0 1];

% Height that we want robot
strikePoseHeight = 80;

% If we are red, then flip direction
if (strcmp('red',color))
    radialVector = radialVector*-1;
end

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
    %if (strcmp('red',color)
        endEff_T_worldFrame = [1 0 0 endEffXYPosInWorldFrame(1);
                                0 1 0 endEffXYPosInWorldFrame(2);
                                0 0 -1 strikePoseHeight;
                                0 0 0 1];
    %else
    %    endEff_T_worldFrame = [-1 0 0 endEffXYPosInWorldFrame(1);
%                                 0 -1 0 endEffXYPosInWorldFrame(2);
%                                 0 0 1 strikePoseHeight;
%                                 0 0 0 1];
    %end
    
    Hw0=[-1,0,0,200;
       0,-1,0,200;
       0,0,1,0;
       0,0,0,1]; 
    
    % Transform end effector T in world frame to robot frame
    endEff_T_robotFrame = Hw0*endEff_T_worldFrame;
    T0e = endEff_T_robotFrame;
    return;
end

% If there is no target block, then output NaN
T0e = NaN;

end

