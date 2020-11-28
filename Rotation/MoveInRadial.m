function [jointvel,configs] = MoveInRadial(q0,endpos,speed,N,plot)
% This function plots a straight line defined by a series of velocity
% vectors.
% Most of the function is pretty much the same as circlePlot so did not
% comment this

% initialize time linespace discretization
t = linspace(0,10,N);
dt = t(2) - t(1);

% initialize velocity values at each time step for circle

v = [1,0,0,0,0,0];
[startpos,T0i]=calculateFK(q0);
start=startpos(6,:);
[discreteLine] = makeLine(start,endpos,5);
v(1:3)=discreteLine(2,:)/norm(discreteLine(2,:));
v=v*speed;

jointvel=zeros(N+1,6);
configs=zeros(N+1,6);
configs(1,:)=q0;

v(3:6) = 0;
jointDesired = 6;
BreakN=1;

if(plot)
    hold on;
    plotJointPos(q0, [0,1,0],4);
end
for i=1:N
    BreakN=i;
    q=configs(i,:);
    % Use IKV to calculate the corresponding qd vector
    qd = IKvelocity(v,q,jointDesired,0);
    qd(6)=0;
    qd=qd';
    jointvel(i+1,:)=qd;
    q = q+qd*dt;
    configs(i+1,:)=q;
    [breaker,T0i]=calculateFK(q);
    breaker=breaker(6,:);
    if(abs(breaker(3)-endpos(3))>5)
        break;
    end
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
if(BreakN<N) 
    jointvel(BreakN:end,:)=[];
    configs(BreakN:end,:)=[];
end
end

