function final(color)
    global lynx % necessary to use ArmController inside a function
    lynx = ArmController(color);
    pause(2);
q=[0,0,0,0,0,0];
q1=[-0.574, 0.566, -0.26, 1.28, 0.85, 20];


 lynx.command(q); 

[q,qd]  = lynx.get_state()
% % %   get state of your opponent's robot 
%[q,qd]  = lynx.get_opponent_state()

end
