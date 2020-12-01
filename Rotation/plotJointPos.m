function [] = plotJointPos(P, color,l)
%PLOTJOINTPOS this function plots an joint config

% Plot the robot links in the start configuration
[jointPositions,T0e]=calculateFK(P);

[row,col] = size(jointPositions);
for joint=1:row-1
    linkPoint1 = jointPositions(joint,:);
    linkPoint2 = jointPositions(joint+1,:);    
    TestPlot(linkPoint1,linkPoint2,1,color,l);
end
end

