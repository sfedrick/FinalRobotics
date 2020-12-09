% dt - time step

% Returns:
% pv - position of end effector using Jacobian FK velocity method
% pq - position of end effector using only position FK

function [pv,pq] = fkeval(dq,q,dt)

% Get the 3x1 linear velocity vector of the end effector at given dq and q.
% This method uses the Jacobian.
vi=FKvelocity(dq,q,6);

% Get the joint positions and homog. transformation matrix at given q. We
% use this to find the current pos of end effector
[Jp,T]=calculateFK(q);
pold=Jp(6,:);

% Use linear vel of end effector and multiply it by our time step, then add
% it to curr pos of end effector to get next position pv.
pv=pold+dt*vi(1:3)';

% We also use another method, which just uses the current position to get 
% the next position of the end effector. This pq value is used to check
% our pv value.
qnew=q+dt*dq;
[Jp,Tp]=calculateFK(qnew);
pq=Jp(6,:);
end

