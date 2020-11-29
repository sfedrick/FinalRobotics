function final(color)
     color = 'blue';
    global lynx % necessary to use ArmController inside a function
    lynx = ArmController(color);

    pause(1)

    % interact with simulator, such as...
     q1=[-1, 0, 0, 0, 0, 0];
    fprintf('\n here \n\n\n');
    lynx.set_pos(q1);
    disp(' there')
    % get state of your robot
    [q,qd]  = lynx.get_state()
    % get state of scoreable objects
   [name,pose,twist] = lynx.get_object_state();

   
    % used to set position to q
    
% % %   get state of your opponent's robot
  %  [q,qd]  = lynx.get_opponent_state()

end
