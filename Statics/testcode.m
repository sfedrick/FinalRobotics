

h = 50;
T =  [ zeros(1,3) , -40; zeros(1,4); 0, 0, 0, 70; zeros(1,4)] + ...
    [0, -1, 0, 110; -1, 0, 0, -285; 0, 0, -1, 60; 0, 0, 0, 1];
[q] = calculateIK(T);

