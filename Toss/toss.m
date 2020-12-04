function [] = toss(color)
%TOSS Summary of this function goes here
     global lynx
     lynx = ArmController(color);
move([-0.8751    0.5806   -0.3087    1.2989    0.0227 30],lynx);
move([-0.8751    0.6518   -0.2440    1.1629    0.0227 30],lynx);
 move([1.15 -1.2 -1.4 0 0 -15],lynx);
             
 lynx.set_vel([0 25 35 50 2.7 100]);
 
end

