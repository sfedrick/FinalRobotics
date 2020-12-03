function [dynamicName, dynamicPose, dynamicTwist] = filterOutStaticBlocks(lynx)
%FILTEROUTSTATICBLOCKS 
% Gets only the dynamic blocks the environment

% initialize vars
dynamicName = cell(5,1);
dynamicPose = cell(5,1);
dynamicTwist = cell(5,1);
count = 1; % for which cell to add to 

% get objects from env
[name,pose,twist] = lynx.get_object_state();

% only get moving boxes
for i=1:length(twist)
    currCell = twist{i};
    if (currCell(6) ~= 0)
        dynamicName{count} = name{i};
        dynamicPose{count} = pose{i};
        dynamicTwist{count} = currCell;
        count = count+1;
    end
end

end

