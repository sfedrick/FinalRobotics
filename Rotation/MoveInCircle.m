function [jointvel,configs] = MoveInCircle(Dradius,speed,N,q0,plot)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% initialize time linespace discretization
t = linspace(0,pi,N);
dt = t(2) - t(1);
%distance from cednter of table in striking pose
r0=100;
r=r0-Dradius;
jointvel=zeros(N+1,6);
configs=zeros(N+1,6);
configs(1,:)=q0;
% initialize radius
%move in radial direction by ammount r0-Dradius
%create function to move radially 

% initialize velocity values at each time step for circle
v = zeros(6,length(t));
for i=1:length(t)
    v(1,i) = r*sin(t(i));
    v(2,i) = r*cos(t(i));
end
% Toggle these on and off to control orientation
%v(6,:)=-1;
%v(1,:) = NaN;
%v(3:6,:) = NaN;
jointDesired = 6;

if(plot)
    hold on;
    circle(200,200,100);
    plotJointPos(q0, [0,1,0],4);
    
end
for i=1:length(v)
    q=configs(i,:);
    % Use IKV to calculate the corresponding qd vector
    qd = IKvelocity(v(1:6,i),q,jointDesired,0);
    qd(6)=0;
    qd=speed*qd';
    jointvel(i+1,:)=qd;
    q = q+qd*dt;
    configs(i+1,:)=q;
    if(plot)
        [jointPos,T] = calculateFK(q);
        
        % Use FKV to get the corresponding linear velocities up until jointDesired
        [pv,pq] = fkeval(qd,q,dt);

        % Plot the orientation vector as well as the actual point
        trajPlot(T,true,'.k');
        trajPlot(pv,false,'.k');
        if(mod(i,10)==0)
            display(T)
        end
    end
end
if(plot)
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    axis equal;
    hold off;
end
end

