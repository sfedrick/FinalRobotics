function [] = trajPlot(T,axisplot,color)
%PLOT plots a given homogeneous matrix in 3D space
if(axisplot)
    % Extract rotation and position matrices
    R = T(1:3,1:3);
    d = T(1:3,4)';

    % Construct x, y, z lines starting at each point
    x = R(1:3,1)';
    y = R(1:3,2)';
    z = R(1:3,3)';
else
    d=T;
end


% Plot actual point in space
plot3(d(1),d(2),d(3),color,'MarkerSize',10); 

% Plots the axes
if(axisplot)
    [Px1,Px2]=longerLines(d,x+d,10);
    [Py1,Py2]=longerLines(d,y+d,10);
    [Pz1,Pz2]=longerLines(d,z+d,10);
    linePlot(Px1,Px2,1,[1,0,0],2); % x = red
    linePlot(Py1,Py2,1,[0,1,0],2); % x = red
    linePlot(Pz1,Pz2,1,[0,0,1],2); % x = red
 
end
end

