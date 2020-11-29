function Tout =  PickedPose(T, base, h)
%%%%%
% input
%          T -- the pose of the block, z axis coresponding to white face.
%          h -- safety height in mm
%          base -- the base coordinate of the robot in ground.
% output
%          Tout--- our desired pose (appropriate pose for robot to pick)

%%%%%

    x0 =  T(1:3, 1);
    y0 = T(1:3, 2);
    z0 = T(1:3, 3);
    p = T(1:3, 4);
    z = [0;0;-1];  % z axis is always downward

    % pick the x axis nearest to the vector from base to p
    t = p - base;
    t = t / norm(t);
    directions = {x0,y0,z0,-x0,-y0,-z0};  % pick from these 6 directions
    [~,index] = min([ norm(x0-t),norm(y0-t),norm(z0-t),norm(-x0-t),norm(-y0-t),norm(-z0-t)]);
    x = directions{index};
    % use cross() to calculate y axis
    y = - cross(x, z);
    Tout = [x, y, z, p + [0;0;h]; 0 0 0 1];


end