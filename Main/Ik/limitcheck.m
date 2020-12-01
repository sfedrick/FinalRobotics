function [output,solution] = limitcheck(joint,upper,lower)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
solution=0;
output=[];
for i=1:length(joint)
    if(~isnan(joint(i))&& isreal(joint(i)))
        if((joint(i)<=upper(i) && joint(i)>= lower(i)))
            solution=solution+1;
        end
    end
end
    if(length(joint)==solution)
        output=joint;
        solution=1;
    else
        solution=0;
    end
end

