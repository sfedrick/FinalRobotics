function final(color)
    
    
    global lynx % necessary to use ArmController inside a function
    lynx = ArmController(color);

    pause(1)
    home = [-0.7 0 0 0 0 0];
    % interact with simulator, such as...
    q1=home;
    fprintf('\n here \n');
    lynx.command(q1);
    disp(' there')
    pause(2)
    % get state of your robot
    [q,qd]  = lynx.get_state()
    [a b] = calculateFK(q)
    
    T=[0, -1, 0, 198; -1, 0, 0, -233; 0, 0, -1, 68; 0, 0, 0, 1];
    [q2 ~] = calculateIK(T)
    
    [c d] = calculateFK(q2)
% 
%     [name,pose,twist] = lynx.get_object_state();
% 
% 
%     [q,qd]  = lynx.get_state()
    % % %   get state of your opponent's robot 
    %[q,qd]  = lynx.get_opponent_state()



end
