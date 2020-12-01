function Tout =  PickedPose(T, pose, base, h)
%%%%%
% input
%          T -- the pose of the block, z axis coresponding to white face.
%          h -- safety height in mm
%          base -- the base coordinate of the robot in ground.
%          pose -- the pose of all the statics blocks
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
    
    % find the closest block.
    dist = [norm(T(1:2,4)-pose{1}(1:2,4)), norm(T(1:2,4)-pose{2}(1:2,4)), ...
            norm(T(1:2,4)-pose{3}(1:2,4)), norm(T(1:2,4)-pose{4}(1:2,4))];
    [~, index] = sort(dist);
    % Note that index(1) is T itself and dist == 0!!
    minDist = dist(index(2)); 
    blockVec = T(1:3,4)-pose{index(2)}(1:3,4);
    blockVec = blockVec / norm(blockVec);
    distol = 40; % mm 
    if minDist < distol
        % In this case, we avoid x axis align with the line between blocks.
        % find closest vec direction to blockVec, which is bad direction.
        directions = [x0,y0,z0,-x0,-y0,-z0];  % pick from these 6 directions
        [~,index2] = min(vecnorm(directions - blockVec, 2, 1));
        badvec = directions(:,index2);
        % abandon the bad vecs and update the directions
        directions = [z, -z, cross(z, badvec), -cross(z, badvec)]; 
        % choose closest direction to the "natural" direction
        [~,index3] = min(vecnorm(directions - t, 2, 1));
        x = directions(:,index3);
        % use cross() to calculate y axis
        y = - cross(x, z);
    else
        directions = [x0,y0,z0,-x0,-y0,-z0];  % pick from these 6 directions
%         [~,index2] = min([ norm(x0-t),norm(y0-t),norm(z0-t),norm(-x0-t),norm(-y0-t),norm(-z0-t)]);
        [~,index4] = min(vecnorm(directions - t, 2, 1));
        x = directions(:,index4);
        % use cross() to calculate y axis
        y = - cross(x, z);
    end
    Tout = [x, y, z, p + [0;0;h]; 0 0 0 1];


end