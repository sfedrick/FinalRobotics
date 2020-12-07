function [angle] = calcSmallerAngleBwTwoVectors(vector1, vector2)
%CALCULATEBLOCKPOSANGLEFROMSTRIKELINE 
% This function calculates the angle that a block is from the end
% effector's strike line vector

    % calculate angle between block and strike line vector in degrees
    angle = acos((vector1*vector2)/(norm(vector2)*norm(vector1)))*180/pi;

    % want the smaller of the two angles, so subtract 90 if greater
    % than 90 b/c acos has 2 solns
    if (angle > 90)
        angle = angle - 90;
    end

end

