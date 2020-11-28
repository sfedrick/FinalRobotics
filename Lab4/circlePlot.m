% This function plots a circle defined by a series of velocity vectors

% initialize time linespace discretization
t = linspace(0,2*pi,50);
dt = t(2) - t(1);

% initialize radius
r = 20;

% initialize velocity values at each time step for circle
v = zeros(6,length(t));
for i=1:length(t)
    v(1,i) = r*sin(t(i));
    v(2,i) = r*cos(t(i));
end

% Toggle these on and off to control orientation
v(6,:)=-1;
%v(1,:) = NaN;
%v(4:6,:) = NaN;

hold on

% Specifies joint desired. If we change this to a lower val, then the
% size of the output qd vector decreases
jointDesired = 6;

% Plot the robot in the zero config
q = [.9, 0, 1, -1, -pi/2]';
[jointPos,T] = calculateFK(q);
for joint=1:5
    linkPoint1 = jointPos(joint,:);
    linkPoint2 = jointPos(joint+1,:);    
    linePlot(linkPoint1,linkPoint2,1,[0,0,0],2);
end

% Loop through velocity vector and plot each end effector pos
for i=1:length(v)
    
    % Use IKV to calculate the corresponding qd vector
    qd = IKvelocity(v(1:6,i),q,jointDesired);
    
    % Used to add extra values in case jointDesired is less than 6.
    for j=1:length(q)
        if (j >= jointDesired)
            qd = [qd; q(j)]; 
        end
    end
    
    % Need to plot the 
    q = q+qd*dt;
    fakeQ = [q; 0];
    [jointPos,T] = calculateFK(fakeQ);
   
    % Use FKV to get the corresponding linear velocities up until jointDesired
    [pv,pq] = fkeval(qd,q,dt);
    
    % Plot the orientation vector as well as the actual point
    trajPlot(T,true,'.k');
    trajPlot(pv,false,'.k');
    if(mod(i,10)==0)
        display(T)
    end

end

xlabel('X');
ylabel('Y');
zlabel('Z');
axis([-260 260 -260 260 0 260]);
axis equal;
hold off