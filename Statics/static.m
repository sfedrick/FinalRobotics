function [] = static(color)
    tic
    global lynx
    lynx = ArmController(color);
    % get state of your robot
    
    pause(2)
    

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
    [name,pose,~] = lynx.get_object_state();
    [a, ~]=size(name);
    
    dist = zeros(a,1);
    for i=1:a
        dist(i,1) = norm(pose{i}(1:2,4) - goal(1:2));
    end
%     dist
    
    priority=zeros(4,1);j=1;i=1;
    while sum(find(priority == 0) > 0) 
        if ((dist(i,1) == min(dist(:,1))) && j < 5)
            priority(j,:)=i;
            j=j+1;
            dist(i,1) = 1000;
            i=1;
        else
            i=i+1;
        end
    end
%     priority
    
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
    [T_pick_g, Flag] = PickedPose(poses{i}, poses, base, h); %desired picked pose in ground frame

    T_pick_r = Trg * T_pick_g  ;           %desired picked pose in robot frame                
    if Flag == 0 
        [T_pick_r, change] = WhiteSideUp(T_pick_r, (Trg * static.pose{i}));
    end
    
    q1 = calculateIK(T_pick_r);
    if isempty(q1)
        T_pick_g = Trg * static.pose{i};
        T_pick_r = [0, 0, 1, T_pick_g(1,4); 0, -1, 0, T_pick_g(2,4); 1, 0, 0, T_pick_g(3,4)+50; 0, 0, 0, 1];
        q1 = calculateIK(T_pick_r);
        q1 = [ q1(1:4), -pi/2];
        disp("basic pose")

    end
    q1 = [q1, 20];
   
    move(q1, lynx)

    
%%%%    T_down_g = T_pick_g - [zeros(3), [0;0;35];0 0 0 0];
    T_down_r = T_pick_r - [zeros(3), [0;0;35];0 0 0 0] ;           %desired picked pose in robot frame                
    qdown = calculateIK(T_down_r);
    
    if isempty(qdown)
        T_pick_g = Trg * static.pose{i};
        T_pick_r2 = [0, 0, 1, T_pick_g(1,4); 0, -1, 0, T_pick_g(2,4); 1, 0, 0, T_pick_g(3,4)-5; 0, 0, 0, 1];
        qdown = calculateIK(T_pick_r2);
        qdown = [qdown(1:3), -0.4, -pi/2];

    end
    qdown = [qdown, 20];
    
    move(qdown, lynx)
    
    %grab the object
    qGrab = [qdown(1:5), -15];

    
    lynx.command(qGrab);
    grab = 0;
    pause(.5)   

    qpick = [q1(1:5), -15];
    

%       NOte that:      pickNorm = norm(q(1:5)-qPick(1:5))
    move(qpick, lynx)
    
        
    if change == 0
        Tplace = [0, -1, 0, 72; -1, 0, 0, -275; 0, 0, -1, 140; 0, 0, 0, 1];
    else 
        Tplace = [1, 0, 0, 69; 0, 0, -1, -270; 0, 1, 0, 140; 0, 0, 0, 1];
   end
    [qPlace, ~] = calculateIK(Tplace);
    qPlace = [qPlace, -15];
    if change == 2
        qPlace = [qPlace(1:4), 1.5, -15];
    end
    move(qPlace, lynx)
    
    % move down to place the block

    if change == 0
       Tdown2 = [0, -1, 0, 72; -1, 0, 0, -275; 0, 0, -1, 30 + i*20 ; 0, 0, 0, 1]; 
    else 
        Tdown2 = [1, 0, 0, 69; 0, 0, -1, -270; 0, 1, 0, (30 + (i*20)); 0, 0, 0, 1];
    end
    %
    qdown2 = calculateIK(Tdown2);
    qdown2 = [qdown2, -15];
    if change == 2
        qdown2 = [qdown2(1:4), 1.5, -15];
    end
    move(qdown2, lynx)
    qDrop = [qdown2(1:5), 30];
    move(qDrop, lynx)
    pause(0.5)
    
    %move to qEnd. qEnd acts as an intermediate position so that
    %the robot doesnot hit a block while moving to another block
  
    Tend = Tdown2 + [zeros(2,4); 0, 0, 0, 60; zeros(1,4)];
    %Tend = [0, -1, 0, 70; -1, 0, 0, -270; 0, 0, -1, 140 ; 0, 0, 0, 1];
    qEnd = calculateIK(Tend);
    qEnd = [qEnd, 30];
    move(qEnd, lynx)
     
     
    disp(" Placed static object ");
end   
toc
end