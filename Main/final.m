function final(color)
    global lynx % necessary to use ArmController inside a function
    lynx = ArmController(color);

    pause(1)

    % interact with simulator, such as...
     q1=[0 0 0 0 0 0];
     fprintf('\n here \n\n\n');
     lynx.set_pos(q1);
    disp(' there')
    pause(2)
    % get state of your robot
    [q,qd]  = lynx.get_state()
    % get state of scoreable objects

   [name,pose,twist] = lynx.get_object_state();

  q=[0, 0, 0, 0, 0, 0];
%velq=[10, 0,0,0,0,0];
%q=[0.759188047744899,0.160342666253201,0.630932408401299,-0.791275074654501,-1.570796326794897,30];
%q=[-0.6340    1.4000   -1.2550    1.5760   -0.4040   -0.4750]; 
%q=[-0.7921    0.6797   -0.4542    1.3453    0.3648   20.0000];
lynx.set_pos(q); % used to set position to q
%lynx.set_vel(velq);
 ToleranceMovement(lynx,q,0.1);
 lynx.set_vel([0,0,0,0,0,0]);

[q,qd]  = lynx.get_state()
% % %   get state of your opponent's robot 
%[q,qd]  = lynx.get_opponent_state()



end
