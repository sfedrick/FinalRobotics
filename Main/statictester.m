function []= statictester(color)
global lynx
lynx=ArmController(color);
pause(2);
static(color)
end

