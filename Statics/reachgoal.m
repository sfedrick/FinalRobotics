function q = reachgoal(coord, color, lynx)
% Calculate desired configuaration given by coordinate and reach to it.
% input:
%        coord -- (x,y,z) coordinate of the goal in ground frame.
%        color -- ...
%        lynx  -- the robot 
% output:
%       q    -- target configuration
%%%%%%%%%%%
   if strcmp(color, 'blue')
       Trg = [-1 0 0 200; 0 -1 0 200; 0 0 1 0; 0 0 0 1];  % transf matrix from ground to robot base.       
   elseif strcmp(color, 'red')
       Trg = [1 0 0 200; 0 1 0 200; 0 0 1 0; 0 0 0 1];    
   else
       error('Sorry, wrong color name!')
   end
    % transfer to ground frame.
    p = reshape(coord, [3,1]);
    p = Trg * [p;1]; % transfer to robot frame.
    T = [0, -1, 0, p(1); -1, 0, 0, p(2); 0, 0, -1, p(3); 0, 0, 0, 1];
    q = calculateIK(T);
    % note that this may not be able to get a solution within joint limits
    % sometime and the code runs into error. So be careful to input the
    % coordinate of the goal.
    q = [q, -15];
    % move to q
    move(q, lynx)
    qDrop = [q(1:5), 30];
    lynx.command(qDrop)
   
end
