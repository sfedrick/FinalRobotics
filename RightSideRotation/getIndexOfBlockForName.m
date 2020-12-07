function [desiredBlockIdx] = getIndexOfBlockForName(blockNames, desiredBlockName)
%GETINDEXOFBLOCKFORNAME 

desiredBlockIdx = NaN;
for i=1:length(blockNames)
    if (strcmp(blockNames{i},desiredBlockName))
        desiredBlockIdx = i;
        return;
    end
end

end

