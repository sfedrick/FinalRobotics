function final(color)
    global lynx % necessary to use ArmController inside a function
    lynx = ArmController(color);
    pause(1);
% 
%    [name,pose,twist] = lynx.get_object_state();
  r=-50;
  %q=findperfect(r);
%q=[-0.747,0.587,-0.281,1.272,-0.586,20];
q=[0.8000    0.6558   -0.4189    1.3339    0 30];
%q=findperfect(r);
%velq=[10, 0,0,0,0,0];
%q=[0.759188047744899,0.160342666253201,0.630932408401299,-0.791275074654501,-1.570796326794897,30];
%q=[-0.6340    1.4000   -1.2550    1.5760   -0.4040   -0.4750]; 
%q=[-0.7921    0.6797   -0.4542    1.3453    0.3648   20.0000];
lynx.set_pos(q); % used to set position to q
%lynx.set_vel(velq);
tic
 ToleranceMovement(lynx,q,0.1,1000);
 lynx.set_vel([0,0,0,0,0,0]);
toc
[q,qd]  = lynx.get_state();
    pause(2);
% 
%    [name,pose,twist] = lynx.get_object_state();
  r=0;
  %q=findperfect(r);
  %lynx.set_vel([0,0,0,0,0,0]);
q=[0,0,0,0,0,0];
%q=[-1.2, 0, -0.2,-0.2,0,30];
velq=[-1, 0,-1,0,0,0];
%q=[0.759188047744899,0.160342666253201,0.630932408401299,-0.791275074654501,-1.570796326794897,30];
%q=[-0.6340    1.4000   -1.2550    1.5760   -0.4040   -0.4750]; 
%q=[-0.7921    0.6797   -0.4542    1.3453    0.3648   20.0000];
lynx.command(q); % used to set position to q
%lynx.set_vel(velq);
pause(2)
%lynx.set_vel([0 0 0 0 0 0]);
tic
 
 %lynx.set_vel([0,-10,-10,0,0,0]);
  %ToleranceMovement(lynx,q,0.1);
toc
[q,qd]  = lynx.get_state();
% % %   get state of your opponent's robot 
%[q,qd]  = lynx.get_opponent_state()



end
