function [Love] = wesleyLovesMe(blockPoses)
%Checks if wesley loves the current configuration of the rotating table
L=length(blockPoses);
Love=false;
for i=1:L
    BoxFrame=blockPoses{i};
    BoxLocation=BoxFrame(1:3,4);
    if(BoxLocation(1)>=0 && BoxLocation(2)>=0)
        Love=true;
    end
    
   
end 
end

