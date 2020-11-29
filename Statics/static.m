function [sDone] = static(color)
    
    global lynx
    lynx = ArmController(color);
    % get state of your robot
    pause(1)
    [q,qd]  = lynx.get_state()

    % get state of scoreable objects
   [name,pose,twist] = lynx.get_object_state();
   
   % define variables
   if strcmp(color, 'blue')
       Trg = [-1 0 0 200; 0 -1 0 200; 0 0 1 0; 0 0 0 1];  % transf matrix from ground to robot base.
       goal = [ 30; 470; 40];
       
       
   elseif strcmp(color, 'red')
       Trg = [1 0 0 200; 0 1 0 200; 0 0 1 0; 0 0 0 1];
       goal = [ 30; 470; 40];  %?? why is not different
       
       
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
    goalTrans= [0, 0, 1, (goal(1)-200); 0, -1, 0, (goal(2)-200); 1, 0, 0, (goal(3) + 50); 0, 0, 0, 1];
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
    celldisp(static.name)
    celldisp(static.pose)
    
    %for i=1:4
    static.pose{1}
    %transforming box into robot's frame

%     T = Trg * static.pose{1};
%     ax = T(1, 4);
%     ay = T(2, 4);
%     az = T(3, 4)+50;
%     Tinput = [0, 0, 1, ax; 0, -1, 0, ay; 1, 0, 0, az; 0, 0, 0, 1];
%     [q, ~] = calculateIK(Tinput)
%     q1 = q(1, 1:4);
%     q1 = [q1, -pi/2, 30];

    %get the coordinate of the robot base in ground
    base = inv(Trg) * [0;0;0;1]; 
    base = base(1:3);
    T_pick_g = PickedPose(static.pose{1}, base, 50); %desired picked pose in ground frame
    T_pick_r = Trg * T_pick_g  ;           %desired picked pose in robot frame                
    q1 = calculateIK(T_pick_r);
    q1 = [q1 30];
    
    lynx.set_pos(q1);
    pause(3)
    [q ~] = lynx.get_state();
    pause(2)
    reach = 0;
    % reach 50 mm above the object
    while (reach == 0)
        [q, ~] = lynx.get_state();
        pause(0.2)
        lynx.set_pos(q1);
        disp("reaching")
        reachNorm = norm(q-q1)
        if norm(q - q1) < 0.5
            reach = 1;
        end
    end
    
    disp("reach complete");
    
    %dive down to the object; adjusted x and y in case the box is aligned
    %in a 'diamond' shape instead of squarely with the end eff.
    T_down_g = T_pick_g - [zeros(3), [0;0;45];0 0 0 0];
    T_down_r = Trg * T_down_g  ;           %desired picked pose in robot frame                
    qdown = calculateIK(T_down_r);
    qdown = [qdown 30];
    lynx.set_pos(qdown);
    dive = 0;
        
    while (dive == 0)
        [q, ~] = lynx.get_state();
%         diveNorm = norm(q - qdown)
        disp("diving")
        if norm(q - qdown) < 0.5
            dive = 1;
            
        end
    end
    disp("Dive complete");
    
    %grab the object
    qGrab = [qdown, -15];
    pause(3)
    lynx.set_pos(qGrab);
    grab = 0;
    pause(5)
    
%     while (grab == 0)
%         [q, ~] = lynx.get_state();
%         if norm(q - qGrab) < 0.05
%             grab = 1;
%         end
%     end
    
    Tpick = Tinput;
    [qpick, ~] = calculateIK(Tpick);
    qPick = [qpick(1:4), -pi/2, -15];
    lynx.set_pos(qPick);
    pick = 0;
    
    while (pick == 0)
        [q, ~] = lynx.get_state();
        disp("picking")
        pickNorm = norm(q-qPick)
        if norm(q - qPick) < 0.5
            pick = 1;
        end
        lynx.set_pos(qPick);
    end
    disp("Pick complete");
    
    move  = 0;
    Tplace = goalTrans
    [qPlace, ~] = calculateIK(goalTrans);
    qPlace(1,6) = -15;
    lynx.set_pos(qPlace)
    
    while (move == 0)
        [q, ~] = lynx.get_state();
        if norm(q - qPlace) < 0.05
            move = 1;
        end
    end
    
    place = 0;
    TDrop = goalTrans * [1, 0, 0, 0; 0, 1, 0, -50; 0, 0, 1, 0; 0, 0, 0, 1];
    [qDrop, ~] = calculateIK(Tdrop);
    qDrop(6,0) = 30;
    lynx.set_pos(qDrop)
    drop = 0;
    
    while (drop == 0)
        [q, ~] = lynx.get_pos();
        if norm(q - qDrop) < -0.05
            drop = 1;
        end
    end
        
    %move back up
    lynx.set_pos(qPlace);
    
    disp(" Placed static object ");
    % Tranformation matrix at goal
    
    
    sDone =1;
   % lynx.set_pos(q);
    
%%% manual testss end
 % %   get state of your opponent's robot
 %   [q,qd]  = lynx.get_opponent_state()
end