% Plots the constant velocity trajectory of the end effector in real space

% We need to specify the initial and final joint configurations for the
% robot. We want it to do a complete rotation about every joint, so we go
% from (0,2*pi)
initial=[0,0,0,0,0,0];
%final=[6.5,6.5,6.5,6.5,6.5,6.5];
final=[3*pi,3*pi,3*pi,3*pi,3*pi,3*pi];

% Linearly interpolate between the initial and final configurations in
% config space with 1000 steps. 
step=1000;
Q=makeLine(initial,final,step);

hold on;

% Plot the robot links in the start configuration for reference
[jointPositions,T0e] = calculateFK(initial);
[row, col] = size(jointPositions);
for joint=1:row-1
    linkPoint1 = jointPositions(joint,:);
    linkPoint2 = jointPositions(joint+1,:);    
    linePlot(linkPoint1,linkPoint2,1,[1,0,0],1);
end

% Using the initial and final configurations, we get the joint angular 
% velocity at every timestep by taking the difference of the first 2
% entries of Q. Remember that we had already discretized this by 1000 steps
dq=Q(2,:)-Q(1,:);

% Remove last entry of dq
dq(end)=[];

% Next, we want to plot the end effector's position at each time step. At
% each given configuration q, we use FK velocity to compute the Jacobian
% for our constant velocity dq. This will give us the position pv of the end
% effector at each config q (plotted in blue .). We also wanted to check that this value was
% correct, so we also used FK position pq to calculate the end effector pos at
% each configuration q (plotted in red *). 
[row,col]=size(Q);
for i=1:row
    q=Q(i,:);
    q(end)=[];
    [pv,pq]=fkeval(dq,q,1/step);
    %trajPlot(pv,false,'.b');
    trajPlot(pq,false,'-.r*');
end
title('Constant Velocity Trajectory of End Effector')
xlabel('X axis (cm)')
ylabel('Y axis (cm)')
zlabel('Z axis (cm)')
axis equal
hold off