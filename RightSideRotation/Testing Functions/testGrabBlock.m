function [] = testGrabBlock()
%TESTGRABBLOCK 
% testing function for grabBlock

desiredBlockName = 'cube_dynamic2';
endEffStrikePose = [-1.0000         0         0  200.0000;
                     0   -1.0000         0  134.9817;
                     0         0   -1.0000   80.0000;
                     0         0         0    1.0000];
color = 'blue';
endEffStrikeLine = [0 1];

grabbedSuccessfully = grabBlock(endEffStrikeLine, desiredBlockName, endEffStrikePose, color);

global lynx;
if (grabbedSuccessfully)
    lynx.set_pos([0 0 0 0 0 -15]);
end

end

