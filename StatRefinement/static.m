function [] = static(color)
    tic
    global lynx
    lynx = ArmController(color);
        
    pause(2)                                                                %set-up time for lynx
    
    [q,qd]  = lynx.get_state();
 
   if strcmp(color, 'blue')                                                 % set goal positions and Tfrmtn frames for both robots
       Trg = [-1 0 0 200; 0 -1 0 200; 0 0 1 0; 0 0 0 1];                    % transf matrix from ground to robot base.
       goal = [ 30; 470; 40];
       goal_trans = [0, -1, 0, 110; -1, 0, 0, -285; 0, 0, -1, 60; 0, 0, 0, 1];
       
   elseif strcmp(color, 'red')
       Trg = [1 0 0 200; 0 1 0 200; 0 0 1 0; 0 0 0 1];
       goal = [ -30; -470; 40];  
       goal_trans = [0, -1, 0, 110; -1, 0, 0, -285; 0, 0, -1, 60; 0, 0, 0, 1];
       
   else
       error('Sorry, wrong color name!')
   end
    h = 50;                                                                  %h is buffer height above the block
    [name,pose,~] = lynx.get_object_state();                                 %h is used while approaching the block
    [a, ~]=size(name);
    
    dist = zeros(a,1);                                                      %store the L2 norm distance of each block from goal
    for i=1:a
        dist(i,1) = norm(pose{i}(1:2,4) - goal(1:2));
    end
    
    priority=zeros(4,1);j=1;i=1;
    while sum(find(priority == 0) > 0)                                      %sort blocks closest to goal pform basis L2 norm
        if ((dist(i,1) == min(dist(:,1))) && j < 5)
            priority(j,:)=i;
            j=j+1;
            dist(i,1) = 1000;
            i=1;
        else
            i=i+1;
        end
    end
    
    for i=1:4                                                               %store the closest 4 blocks from goal;
        static.name{i,1} = name{priority(i,:)};                             %name and pose stored in 'priority'
        static.pose{i,1} = pose{priority(i,:)};
    end

 for i=1:4

    % update the position of the block in every loop.
    [namelist,poselist,~] = lynx.get_object_state();
    poses = getpose(static.name, poselist, namelist);
    base = inv(Trg) * [0;0;0;1] ;
    base = base(1:3);
    T_pick_g = PickedPose(poses{i}, poses, base, h);                        %desired picked pose in ground frame

    T_pick_r = Trg * T_pick_g  ;                                            %desired picked pose in robot frame                
    q1 = calculateIK(T_pick_r);                                             %all picked poses are with gripr-vertical orientation
    if isempty(q1)
        T_pick_g = Trg * static.pose{i};
        T_pick_r = [0, 0, 1, T_pick_g(1,4); 0, -1, 0, T_pick_g(2,4); 1, 0, 0, T_pick_g(3,4)+50; 0, 0, 0, 1];
        q1 = calculateIK(T_pick_r);
        q1 = [ q1(1:4), -pi/2];
        disp("basic pose")

    end
    q1 = [q1, 20];
   
    move(q1, lynx)                                                          %move is defined function

    
    T_down_g = T_pick_g - [zeros(3), [0;0;35];0 0 0 0];
    T_down_r = Trg * T_down_g  ;                                            %desired picked pose in robot frame                
    qdown = calculateIK(T_down_r);
    
    if isempty(qdown)                                                       %if out of limit, approach box in gripr-horizontal orientation
        T_pick_g = Trg * static.pose{i};
        T_pick_r2 = [0, 0, 1, T_pick_g(1,4); 0, -1, 0, T_pick_g(2,4); 1, 0, 0, T_pick_g(3,4)-5; 0, 0, 0, 1];
        qdown = calculateIK(T_pick_r2);
        qdown = [qdown(1:3), -0.4, -pi/2];

    end
    qdown = [qdown, 20];                                                    %open fingers to q(6) = +20
    
    move(qdown, lynx)
    
    %grab the object
    qGrab = [qdown(1:5), -15];                                              %grab object as tight as possible, q(6) = -15

    
    lynx.command(qGrab);
    grab = 0;
    pause(1)   

    qpick = [q1(1:5), -15];
    move(qpick, lynx)                                                       %move is a defined function
    
    %stacks of more than 2 are risky as any slight movement by robot
    %will make them fall. Hence, made 2 different positions for
    %two stacks of 2 static blocks
    if i<3
        Tplace = goal_trans + [ zeros(1,3), -15; zeros(1,4); 0, 0, 0, 60; zeros(1,4)];
    else
        Tplace = goal_trans + [ zeros(1,3), +10; zeros(1,4); 0, 0, 0, 60; zeros(1,4)];
    end
    [qPlace, ~] = calculateIK(Tplace);
    qPlace = [qPlace, -15];
    
    move(qPlace, lynx)
    
    if i<3                                                                  %defines height of stacking; 2 block is 20mm above first
        Tdown2 = Tplace - [ zeros(2,4); 0, 0, 0, (60 - i*20); zeros(1,4)];
    else
        Tdown2 = Tplace - [ zeros(2,4); 0, 0, 0, (50 - (i-2)*20) ;zeros(1,4)];
    end
    qdown2 = calculateIK(Tdown2);
    qdown2 = [qdown2, -15];
    move(qdown2, lynx)
    qDrop = [qdown2(1:5), 30];
    move(qDrop, lynx)
    pause(0.5)
    
    qEnd = [-1.2, 0, 0.2, 0, qDrop(5:6)];                                   %move to qEnd. qEnd acts as an intermediate position so that
                                                                            %the robot doesnot hit a block while moving to another block
    %lynx.set_vel([1,0,0,0,0,0]);                                           %this is done by retracting the robot upwards towards qEnd
    
     move(qEnd, lynx)
     
    disp(" Placed static object ");
 end   
toc
end