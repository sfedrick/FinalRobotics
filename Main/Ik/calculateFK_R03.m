function [jointPositions,R03] = calculateFK_R03(q)
% CALCULATEFK - 
%
% DO NOT MODIFY THE FUNCTION DECLARATION
%
% INPUT:
%   q - 1x6 vector of joint inputs [q1,q2,q3,q4,q5,lg]
%
% OUTPUT:
%   jointPositions - 6 x 3 matrix, where each row represents one 
%                    joint along the robot. Each row contains the [x,y,z]
%                    coordinates of the respective joint's center (mm). For
%                    consistency, the first joint should be located at 
%                    [0,0,0]. These values are used to plot the robot.
%   T0e            - a 4 x 4 homogeneous transformation matrix, 
%                    representing the end effector frame expressed in the 
%                    base (0) frame
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Lynx Dimensions in mm
L1 = 76.2;    % distance between joint 0 and joint 1
L2 = 146.05;  % distance between joint 1 and joint 2
L3 = 187.325; % distance between joint 2 and joint 3
L4 = 34;      % distance between joint 3 and joint 4
L5 = 34;      % distance between joint 4 and center of gripper

%% Your code here
joint1 = q(1);
joint2 = q(2);
joint3 = q(3);
% joint4 = q(4);
% joint5 = q(5);
jointPositions=zeros(4,3);
R03 = eye(4,4);
T = eye(4);
DH_params =[0, -pi/2, L1, joint1; 
            L2, 0, 0, joint2-pi/2;
            L3, 0, 0, joint3+pi/2;];
%            0, -pi/2, 0, joint4 - pi/2;
  %          0, 0, L4+L5, joint5];

        %Calculate each intermediate homogeneous transformation matrix
for link = 1:3
    a = DH_params(link,1);
    alpha = DH_params(link,2);
    d = DH_params(link,3);
    theta = DH_params(link,4);

    % calculate A
    A = createA(a, alpha, d, theta);
    T = T*A;
    jointPositions(link+1,:) = T(1:3,4)';
   
end

R03 = T(1:3,1:3);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end