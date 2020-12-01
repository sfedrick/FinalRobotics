function [] = rotationdriver(color)

    %% Simulation Parameters

    %start = [.9, 0, 1, -1, -pi/2,30];
    %start=[0.75, 0.1, 0.9, -1, -pi/2,30];
    %start=[0.75, 0.1, 0.7, -0.8, -pi/2,30];
    r=25;
    start=findperfect(r);
    
   error=0.1;
   derror=0.1;
    % Find collision-free path using RRT to get list of waypoints
    
    %[jointvel,configs] = MoveInRadial(start,[200,200,68],speed,N,0);
    
    %[row,col]=size(jointvel);
    %[path] = astar(map, start, goal);

    %start ROS
     global lynx
    lynx = ArmController(color);
    pause(1);% wait for setup
    %collision = false;
    
    lynx.set_pos(start);
    ToleranceMovement(lynx,start,error);

    lynx.set_vel([0,0,0,0,0,0])
    [q,qd]  = lynx.get_state()
    
    speed=1.5;
    [jointvel,configs] = MoveInCircle(r,speed,speed*100,q,1);
    qold=q;
tic
   for target_index = 1:length(jointvel(:,1))
        dq = jointvel(target_index, :);
        q = configs(target_index,:);
        disp("Goal:")
        disp(dq)
        lynx.set_vel(dq)
    
        
         ToleranceMovement(lynx,q,error+derror);
        [WithInFace,BoxesInUrFace] = InUrFace(lynx,color,50,0.7,3);
        
        [Stop] = QuarterGrab(q,qold,Error);
        
        if(WithInFace||Stop)
            lynx.set_vel([0,0,0,0,0,0]);
            break;
        end
        disp("Current Configuration:");
        [pos, ~] = lynx.get_state()
        qold=pos;
   end
  toc  
%     display("finshed");
%     lynx.set_vel([0,0,0,0,0,0])
%     if collision
%         disp("Robot collided during move")
%     else
%         disp("No collision detected")
%     end

    lynx.stop() % Shut down ROS interface
end
