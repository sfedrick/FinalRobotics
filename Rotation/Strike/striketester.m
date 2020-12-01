color='blue';
global lynx 
lynx=ArmController(color);
pause(1)
tic
%cube static 2
%cube static 6
Range=100;
Direction=0.7;
Axis=3;
[WithInFace,BoxesInUrFace] = InUrFace(lynx,color,Range,Direction,Axis);
toc