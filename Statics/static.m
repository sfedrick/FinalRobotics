function [] = static(color)
    color='blue';
    global lynx
    lynx = ArmController(color);
    % get state of your robot
    
    pause(2)
    
    [q,qd]  = lynx.get_state()

    % get state of scoreable objects
   [name,pose,twist] = lynx.get_object_state();
   
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
%     name
%     celldisp(pose)
%     celldisp(twist)

%     for i=1:13
%         if (sum(twist{i}) == 0)
%             st(i,:) = 1;
%         end
%     end
%     [n ~]=find(st > 0);
%     
    %find euclidean distance between goal and closes 4 objects
    [q0, ~] = lynx.get_state();
    %goalTrans= [0, 0, 1, (goal(1)-200); 0, -1, 0, (goal(2)-200); 1, 0, 0, (goal(3) + 50); 0, 0, 0, 1];
    goal_b = [0, -1, 0, 110; -1, 0, 0, -285; 0, 0, -1, 100; 0, 0, 0, 1];
    %for blue only
    [a ~]=size(name);
    
    dist = zeros(a,1);
    for i=1:a
        dist(i,1) = norm(pose{i}(1:2,4) - goal(1:2));
    end
    dist
    
    priority=zeros(4,1);j=1;i=1;
    while sum(find(priority == 0) > 0) 
        if ((dist(i,1) == min(dist(:,1))) & j < 5)
            priority(j,:)=i;
            j=j+1;
            dist(i,1) = 1000;
            i=1;
        else
            i=i+1;
        end
    end
    priority
    
    for i=1:4
        static.name{i,1} = name{priority(i,:)};
        static.pose{i,1} = pose{priority(i,:)};
    end
    %picking static block
%      celldisp(static.name)
%     celldisp(static.pose)
    

%     Tinput = [0, 1, 0, ax; 0, -1, 0, ay; -1, 0, 0, az; 0, 0, 0, 1];
 for i=1:4
    %get the coordinate of the robot base in ground
    base = inv(Trg) * [0;0;0;1] 
    base = base(1:3);
    T_pick_g = PickedPose(static.pose{i}, base, 50); %desired picked pose in ground frame
    T_pick_r = Trg * T_pick_g  ;           %desired picked pose in robot frame                
    q1 = calculateIK(T_pick_r);
    q1 = [q1, 20]
    
    lynx.command(q1);
    pause(1)
    [q ~] = lynx.get_state();
    
    reach = 0;
    % reach 50 mm above the object
    while (reach == 0)
        [q, ~] = lynx.get_state()
        pause(0.1)
        lynx.command(q1)
        disp("reaching")
        reachNorm = norm(q-q1)
        if reachNorm < 0.1
            reach = 1;
        else
            lynx.command(q1)
        end
    end
    
    disp("reach complete");
    pause(0.5)
    
    T_down_g = T_pick_g - [zeros(3), [0;0;35];0 0 0 0];
    T_down_r = Trg * T_down_g  ;           %desired picked pose in robot frame                
    qdown = calculateIK(T_down_r);
    qdown = [qdown 20];
    lynx.command(qdown);
    pause(1)
    dive = 0;
        
    while (dive == 0)
        [q, ~] = lynx.get_state()
         diveNorm = norm(q - qdown)
        disp("diving")
        if norm(q - qdown) < 0.2
            dive = 1;
        else
            lynx.command(qdown);
        end
    end
    disp("Dive complete");
    pause(1)
    %grab the object
    qGrab = [qdown(1:5), -15];
    
    lynx.command(qGrab);
    grab = 0;
    pause(1)
    

    
    Tpick = T_pick_r;
    [qpick, ~] = calculateIK(Tpick);
    qPick = [qpick, -15]
    %qPick
    lynx.command(qPick);
    pause(0.5)
    pick = 0;
    while (pick == 0)
        [q, ~] = lynx.get_state()
        disp("picking")
        qPick
        pickNorm = norm(q(1:5)-qPick(1:5))
        if pickNorm < 0.5
            pick = 1;
        else
        lynx.command(qPick);
        end
    end
    disp("Pick complete");
    
    %stacks of more than 2 are risky as any slight movement by robot
    %will make them fall. Hence, made 2 different positions for
    %two stacks of 2 static blocks
    Move  = 0; goal_trans
    if i<3
        Tplace = goal_trans + [ zeros(1,3), -40; zeros(1,4); 0, 0, 0, (i * 20); zeros(1,4)];
    else
        Tplace = goal_trans + [ zeros(1,3), +10; zeros(1,4); 0, 0, 0, ((i-2) * 20); zeros(1,4)];
    end
    [qPlace, ~] = calculateIK(Tplace);
    qPlace = [qPlace, -15];
    lynx.command(qPlace)
    pause(2)
    while (Move == 0)
        [q, ~] = lynx.get_state();
        qPlace
        moveNorm = norm (q(1:5) - qPlace(1:5))
        if moveNorm < 0.6
            Move = 1;
%         else
%             lynx.command(qPlace);
        end
    end
    
    disp("Move complete");
    TDrop = goal_trans;
    [qDrop, ~] = calculateIK(TDrop);
    qDrop = [qPlace(1:5), 30];
    pause(1)
    lynx.command(qDrop)
    pause(1)
    drop = 0;
    
    while (drop == 0)
        [q, ~] = lynx.get_state();
        normDrop = norm(q - qDrop)
        if normDrop < 0.5
            drop = 1;
        else
            lynx.command(qDrop);
        end
    end
    
    disp("Object Dropped");
    %move to qEnd. qEnd acts as an intermediate position so that
    %the robot doesnot hit a block while moving to another block
    %this is done by retracting the robot upwards towards qEnd
    qEnd = [0, 0,qPlace(3:5), 30];
    lynx.command(qEnd);
    pause(1)
    
    disp(" Placed static object ");
    % Tranformation matrix at goal
    
    
    sDone =1;
   % lynx.set_pos(q);
 end   
%%% manual testss end
 % %   get state of your opponent's robot
 %   [q,qd]  = lynx.get_opponent_state()
end