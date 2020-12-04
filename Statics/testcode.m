
addpath('../Main');
addpath('../Main/Core')
% lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0, -15]; % Lower joint limits in radians (grip in mm (negative closes more firmly))
% upperLim = [ 1.4,  1.4,  1.7,  1.7,  1.5,  30]; % Upper joint limits in radians (grip in mm)
h = 50;
% T =  [ zeros(1,3) , -40; zeros(1,4); 0, 0, 0, 80; zeros(1,4)] + ...
%     [0, -1, 0, 110; -1, 0, 0, -285; 0, 0, -1, 60; 0, 0, 0, 1];
T1 = [0, -1, 0, 60; -1, 0, 0, -260; 0, 0, -1, 150; 0, 0, 0, 1];
T2 = [0, -1, 0, 70; -1, 0, 0, -270; 0, 0, -1, 140; 0, 0, 0, 1];
T3 = [0, -1, 0, 80; -1, 0, 0, -280; 0, 0, -1, 130; 0, 0, 0, 1];
Tour = [0, -1, 0, 95; -1, 0, 0, -285; 0, 0, -1, 120; 0, 0, 0, 1];
[q] = calculateIK(Tour);
disp(q)


