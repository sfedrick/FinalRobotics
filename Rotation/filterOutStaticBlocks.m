function [dynamicName, dynamicPose, dynamicTwist] = filterOutStaticBlocks(lynx)
%FILTEROUTSTATICBLOCKS 
% Gets only the dynamic blocks the environment

% initialize vars
dynamicName = {};
dynamicPose = {};
dynamicTwist = {};
count = 1; % for which cell to add to 

% get objects from env
[name,pose,twist] = lynx.get_state();

% only get moving boxes
for i=1:length(twist)
    currCell = twist{i};
    if (currCell(6) ~= 0)
        dynamicName{count} = name{i};
        dynamicPose{count} = pose{i};
        dynamicTwist{count} = twist{i};
        count = count+1;
    end
end

end

