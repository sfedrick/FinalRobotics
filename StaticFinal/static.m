function [] = static(color)
    tic
    global lynx
    lynx = ArmController(color);
    % get state of your robot
    pause(2)
    %lynx.wait_for_start()
    

    [q,qd]  = lynx.get_state();
 

   % define variables
   if strcmp(color, 'blue')
       Trg = [-1 0 0 200; 0 -1 0 200; 0 0 1 0; 0 0 0 1];  % transf matrix from ground to robot base.
       goal = [ 30; 470; 40];
       goal_trans = [0, -1, 0, 110; -1, 0, 0, -285; 0, 0, -1, 60; 0, 0, 0, 1];
       
   elseif strcmp(color, 'red')
       Trg = [1 0 0 200; 0 1 0 200; 0 0 1 0; 0 0 0 1];
       goal = [ -30; -470; 40];  
       goal_trans = [0, -1, 0, 110; -1, 0, 0, -285; 0, 0, -1, 60; 0, 0, 0, 1];
       
   else
       error('Sorry, wrong color name!')
   end
   h = 50;   
    [name,pose,twist] = lynx.get_object_state();
    [a, ~]=size(name);
    
    dist = zeros(a,1);
    for i=1:a
        dist(i,1) = norm(pose{i}(1:2,4) - goal(1:2));
    end

    
    priority=zeros(4,1);j=1;i=1;
    while sum(find(priority == 0) > 0) 
        if ((dist(i,1) == min(dist(:,1))) && j < 5 && (sum(twist{i}) == 0))
            priority(j,:)=i;
            j=j+1;
            dist(i,1) = 1000;
            i=1;
        else
            i=i+1;
        end
    end

    
    for i=1:4
        static.name{i,1} = name{priority(i,:)};
        static.pose{i,1} = pose{priority(i,:)};
    end

 for i=1:4

    % update the position of the block in every loop.
    [namelist,poselist,~] = lynx.get_object_state();
    poses = getpose(static.name, poselist, namelist);
    base = inv(Trg) * [0;0;0;1] ;
    base = base(1:3);
    
%%%% Move above block %%%%   

    [T_pick_g, Flag] = PickedPose(poses{i}, poses, base, h);                % Picked pose helps in picking block where its sides are square to the fingers of the gripper
                                                                            % Flag is used to mark that the block is too close to another block - so not to orient white side up
    T_pick_r = Trg * T_pick_g;                                              % desired picked pose in robot frame                
    q1prior = calculateIK(T_pick_r);                                        % q1prior is used if there is nor solution for WhiteSideup()
    T_pick_prior = T_pick_r;
    if Flag == 0 
         [T_pick_r, change] = WhiteSideUp(T_pick_r, (Trg * static.pose{i}));
     else
         change = 0;
     end
    placeFlag = 0;
    q1 = calculateIK(T_pick_r);
    if isempty(q1)
        q1 = q1prior;                                                       % if no soln for whiteSideUp(), then use q1 prior to pick up sqaured to sides
        if isempty(q1)                                                      % If no soln for q1Prior, then pick it up in whatever way possible
            T_pick_g = Trg * static.pose{i};
            T_pick_r = [0, 0, 1, T_pick_g(1,4)-5; 0, -1, 0, T_pick_g(2,4)+8; 1, 0, 0, T_pick_g(3,4)+50; 0, 0, 0, 1];
            q1 = calculateIK(T_pick_r); placeFlag = 1;
            if (abs(q1(5)) < 1.4)
            q1 = [ q1(1:4), -pi/2];
            end
        end
    end
    q1 = [q1, 20];
   
    move(q1, lynx); 
%%%% Move down to block position %%%%

    T_down_r = T_pick_r - [zeros(3), [0;0;35];0 0 0 0];                     % desired picked pose in robot frame                
    qdown = calculateIK(T_down_r);
   % placeFlag = 0;                                                          % used to mark an infeasible pick
    if isempty(qdown) | any(T_down_r(1:3, 3) ~= [0; 0;-1])                  % also check whether horizontal pick or vertical pick
        T_down_r1 = T_pick_prior - [zeros(3), [0;0;35];0 0 0 0];            % we prefer vertical pick as that will help
        qdown = calculateIK(T_down_r1);                                     % us to align white side up during placement
        if isempty(qdown) | any(T_down_r1(1:3,3) ~= [0;0;-1])
            T_down_g = static.pose{i} + [zeros(3), [0;0;20];0 0 0 0];       % Preferable pick position is
            T_down_rr = Trg * T_down_g;                                     % WhiteSide up position / Vertical gripper
            qdown = calculateIK(T_down_rr);                                 % then, block face squared with gripper / Vertical
            if isempty(qdown) | placeFlag == 1                              % last resort is horizontal gripper / cannot align whiteSide up
                T_pick_g = Trg * static.pose{i};
                T_pick_r2 = [0, 0, 1, T_pick_g(1,4)-7; 0, -1, 0, T_pick_g(2,4)+10; 1, 0, 0, T_pick_g(3,4)-5; 0, 0, 0, 1];
                qdown = calculateIK(T_pick_r2);
                qdown = [qdown(1:3), -0.4, -pi/2];
                placeFlag = 1; %used to alter place in case of infeasible picks
            end
        end
    end
    qdown = [qdown, 20];
    if placeFlag == 1
        qdown(6) = 30;
    end
    move(qdown, lynx)
    
%%%% Grab the block %%%%

    qGrab = [qdown(1:5), -15];
    move(qGrabm lynx); %lynx.command(qGrab);
    %pause(.5)   

%%%% Pick the block up after grabbing %%%%    

    qpick = [q1(1:5), -15];
    move(qpick, lynx)
    
%%%% Move above the goal position %%%%

    if change == 0
        Tplace = [0, -1, 0, 72; -1, 0, 0, -275; 0, 0, -1, 140; 0, 0, 0, 1];
    else
        Tplace = [1, 0, 0, 53; 0, 0, -1, -270; 0, 1, 0, 140; 0, 0, 0, 1];
    end

    [qPlace, ~] = calculateIK(Tplace);
    
% % This part reduces chances of slip as total rotn is divided in 2 parts % % 
    if change == 2
       qRotate = [qpick(1:4), qPlace(5), -15];
       move(qRotate, lynx)
       qPlace(5) = 1.5;
    end
    
    
    qPlace = [qPlace, -15];
    move(qPlace, lynx)
    
%%%% move down to place the block %%%%

    if change == 0
       Tdown2 = [0, -1, 0, 72; -1, 0, 0, -275; 0, 0, -1, (30 + (i*21)); 0, 0, 0, 1]; 
    else 
       Tdown2 = [1, 0, 0, 53; 0, 0, -1, -270; 0, 1, 0, (30 + (i*21)); 0, 0, 0, 1];
    end
    if placeFlag == 1                                                       % placeFlag is used for infeasible-picks  
       Tdown2(2,4) = -273;                                                  % box may or may not be squared with gripper
    end                                                                     % hence, adjusting placement along y axis only
    
    qdown2 = calculateIK(Tdown2);
    qdown2 = [qdown2, -15];
    if change == 2
        qdown2 = [qdown2(1:4), 1.5, -15];
    end
    move(qdown2, lynx)
    qDrop = [qdown2(1:5), 30];
    move(qDrop, lynx)
   % pause(0.5)
    
%%%% move to qEnd; after place complete %%%%
  
    Tend = Tdown2;                                                          % qEnd is at a safe height
    Tend(3, 4) = 140;                                                       % safe hieght = 140mm through trial and error
    qEnd = calculateIK(Tend);
    
    if abs(qEnd(5)) > 1.4                                                   % we donot want roll while moving up
        qEnd(5) = qDrop(5);                                                 % as roll may topple the stacked blocks
    end
    qEnd = [qEnd, 30];
    move(qEnd, lynx)
       
    disp(" Placed static object ");
end   
toc
end