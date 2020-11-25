function final(color)
%     color = 'blue';
    global lynx % necessary to use ArmController inside a function
    lynx = ArmController(color);

    pause(1)

    % interact with simulator, such as...

    % get state of your robot
    [q,qd]  = lynx.get_state()

    % get state of scoreable objects
   [name,pose,twist] = lynx.get_object_state();

%    q=[0.9, pi/4, -0.3, -0.3, -pi/2, 0];
    lynx.set_pos(q); % used to set position to q
    
% % %   get state of your opponent's robot
  %  [q,qd]  = lynx.get_opponent_state()

end
