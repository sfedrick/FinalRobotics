function [] = linePlot(point1, point2, hasLine, color, width)
% Plots line between 2 nodes and also plots the 2nd node

hold on;

if (hasLine == true)
    
    v1=point1;
    v2=point2;
    v=[v2;v1];
    plot3(v(:,1),v(:,2),v(:,3),'Color',color,'linewidth',width);

end

end

