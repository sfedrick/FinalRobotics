function final(color)
    % color = 'blue';
    global lynx % necessary to use ArmController inside a function
    lynx = ArmController(color);

    pause(1)

    % interact with simulator, such as...

    % get state of your robot
    [q,qd]  = lynx.get_state()

    % get state of scoreable objects
   [name,pose,twist] = lynx.get_object_state();

  %q=[0.9, pi/4, -0.3, -0.3, -pi/2, 0];
q=[1.1, 0.1, 0.7, -0.8, -pi/2,30];
%q=[0.759188047744899,0.160342666253201,0.630932408401299,-0.791275074654501,-1.570796326794897,30];
%q=[0,0,0,0,0,0]; 
lynx.set_pos(q); % used to set position to q
 ToleranceMovement(lynx,q,0.1)
[q,qd]  = lynx.get_state()
% % %   get state of your opponent's robot
  %  [q,qd]  = lynx.get_opponent_state()


end
