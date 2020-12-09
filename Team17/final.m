function final(color)

global lynx;
lynx = ArmController(color);
pause(1);
%lynx.wait_for_start();
dynamics(color);
static(color);
end

    
