color='blue';
global lynx 
lynx=ArmController(color);
pause(1)
tic
%cube static 2
%cube static 6
[WithInFace,BoxesInUrFace] = InUrFace(lynx,color);
toc