function [] = rotationdriver(color)

    %% Simulation Parameters

    start = [.9, 0, 1, -1, -pi/2,30];
    speed = 30;
    N = 300;

    % Find collision-free path using RRT to get list of waypoints
    %[jointvel,configs] = MoveInCircle(Dradius,speed,N,start,plot);
    [jointvel,configs] = MoveInRadial(start,[200,200,64],speed,N,1);
    [row,col]=size(jointvel);
    %[path] = astar(map, start, goal);

    %start ROS
     global lynx
    lynx = ArmController(color);
    pause(1);% wait for setup
    %collision = false;
    
    lynx.set_pos(start);
    pause(1) 

    % iterate over target waypoints
    for target_index = 1:length(jointvel(:,1))
        dq = jointvel(target_index, :);
        q = configs(target_index,:);
        disp("Goal:")
        disp(dq)
        lynx.set_vel(dq)
        reached_target = false;

        % Define relevant variables here:
        if(mod(target_index,10)==0)
            disp("Percent of path completed")
            percentcomplete=target_index/row
        end
        while ~reached_target
            % Check if robot is collided then wait

            %collision = collision | lynx.is_collided();
            pause(0.05)
            error=0.2;
            % Add Student code here to decide if controller should send next
            % target or continue to wait. Do NOT add additional pauses to control
            % loop. You will likely want to use lynx.get_state() to decide when to
            % move to the next target.
            [pos, vel] = lynx.get_state();
            %disp("position difference")
            posDiff=norm(pos(1:5)-q(1:5));
            if(posDiff<error)
                reached_target = true;
            end

            % End of student code
        end
        % End control loop

        disp("Current Configuration:");
        [pos, vel] = lynx.get_state();
        disp(pos);
    end
%     if collision
%         disp("Robot collided during move")
%     else
%         disp("No collision detected")
%     end

    lynx.stop() % Shut down ROS interface
end
