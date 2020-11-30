function [] = TestPlot(nodeOld, nodeNew, hasLine, color, width)
% Plots line between 2 nodes and also plots the 2nd node

hold on;
plot3(nodeNew(1),nodeNew(2),nodeNew(3),'.k','MarkerSize',30); 
if (hasLine == true)
    line = nodeOld-nodeNew;
    step = line/1000;
    
    v1=nodeOld;
    v2=nodeNew;
    v=[v2;v1];
    plot3(v(:,1),v(:,2),v(:,3),'Color',color,'linewidth',width);

end

end

