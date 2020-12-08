function poses = getpose(name, poselist, namelist)
% find the block from namelist using the name.
% name, namelist and poselist are all cells!!
    n = length(name);
    list = length(namelist);
    for i = 1:n
        for j = 1:list
            if strcmp(name{i}, namelist{j})
                poses{i} = poselist{j};
                break;
            end
        end
    end
end