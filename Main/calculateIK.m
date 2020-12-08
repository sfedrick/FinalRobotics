function [q,isPos] = calculateIK(T0e)
% CALCULATEIK - Please rename this function using your group # in
%   both the function header and the file name. 
%
% DO NOT MODIFY THE FUNCTION DECLARATION
%
% INPUT:
%   T - 4 x 4 homogeneous transformation matrix, representing 
%               the end effector frame expressed in the base (0) frame
%               (position in mm)
%
% OUTPUT:
%   q          - a n x 5 vector of joint inputs [q1,q2,q3,q4,q5] (rad) 
%                which are required for the Lynx robot to reach the given 
%                transformation matrix T. Each row represents a single
%                solution to the IK problem. If the transform is
%                infeasible, q should be all zeros.
%   isPos      - a boolean set to true if the provided
%                transformation T is achievable by the Lynx robot, ignoring
%                joint limits
% 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

d1 = 76.2;                      % Distance between joint 1 and joint 2
a2 = 146.05;                    % Distance between joint 2 and joint 3
a3 = 187.325;                   % Distance between joint 3 and joint 4
d4 = 34;                        % Distance between joint 4 and joint 5
d5 = 68;                        % Distance between joint 4 and end effector

% Joint limits
lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0, -15]; % Lower joint limits in radians (grip in mm (negative closes more firmly))
upperLim = [ 1.4,  1.4,  1.7,  1.7,  1.5,  30]; % Upper joint limits in radians (grip in mm)

%% Your code here

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



o=T0e(1:3,4);
q = [0 0 0 0 0];
%finding wrist location in 'oc'

oc=[o(1,:)-(d5*T0e(1,3));
    o(2,:)-(d5*T0e(2,3));
    o(3,:)-(d5*T0e(3,3))];

%oc should within the reach of RR robot formed by theta2 and theta3

oc_o1 = oc - [0;0;d1];
if norm(oc_o1,2) > a2 + a3
    q = [];fprintf('\n');
    isPos = 0; 
    return 
end

%determine the feasibility of orientation
%Note that theta1 is fixed by the position, only theta4 and theta5 dedicate to the end pose orientation.
fprintf('\n');
% disp('---checking feasibility:---');

isPos = checkfeasibility(T0e);          %defined in line 200-218


% if ~isPos
%     disp('-- After correction --')
%     T0e = corrected(T0e);               %defined in line 219-254
% end
% update oc o
o = T0e(1:3,4);
oc=[o(1,:)-(d5*T0e(1,3));
    o(2,:)-(d5*T0e(2,3));
    o(3,:)-(d5*T0e(3,3))];
%finding theta1, theta2 and theta3, based on Pre-Lab2 calculations

%Top View parameters
%We discuss 2 cases according to the value of theta1
%% theta1_1 case
theta1= atan2(oc(2,1),oc(1,1));
r1=sqrt((oc(1,1)^2)+(oc(2,1)^2));

%Side View parameters

r3=oc(3,1)-d1;
r2=sqrt((r1^2)+(r3^2));
d=((r2^2)-(a2^2)-(a3^2))/(2*a2*a3);
phi2=atan2(sqrt(1-(d^2)),d);
phi2_2 = -phi2;
phi3=atan2(r3,r1);
phi1=atan2((a3*sin(phi2)),(a2+(a3*cos(phi2))));
phi1_2 = atan2((a3*sin(phi2_2)),(a2+(a3*cos(phi2_2))));

%Finding theta3 and theta2 basis side view parameters defined above

theta3=phi2-pi/2;
theta3_2 = phi2_2 - pi/2;
theta2=pi/2-phi1-phi3;
theta2_2 = pi/2 - phi1_2 -phi3;
%% theta1_2 case

theta1_2= wrapTo2Pi(theta1 + pi);  %second solution for theta1
r1 = - sqrt((oc(1,1)^2)+(oc(2,1)^2));  % pick negative r1;
%Actually r1 only affect theta2
% r3=oc(3,1)-d1;
% r2=sqrt((r1^2)+(r3^2));
% d=((r2^2)-(a2^2)-(a3^2))/(2*a2*a3);
% phi2=atan2(sqrt(1-(d^2)),d);
% phi2_2 = -phi2;
phi3=atan2(r3,r1);
% phi1=atan2((a3*sin(phi2)),(a2+(a3*cos(phi2))));
% phi1_2 = atan2((a3*sin(phi2_2)),(a2+(a3*cos(phi2_2))));

%Finding theta3 and theta2 basis side view parameters defined above

theta3_3 = phi2-pi/2;                        %same as theta3
theta3_4 = phi2_2 - pi/2;                    %same as theta3_2
theta2_3 = pi/2-phi1-phi3;                   %same as theta2
theta2_4 = pi/2 - phi1_2 -phi3;              %same as theta2_2
%%
%conclude 4 solutions of theta 1-3:
%sol_i = [theta1_i theta2_i theta3_i]
sol1 = [theta1 theta2 theta3];
sol2 = [theta1 theta2_2 theta3_2];
sol3 = [theta1_2 theta2_3 theta3_3];
sol4 = [theta1_2 theta2_4 theta3_4];

AllSol_1to3 = {sol1 sol2 sol3 sol4};
%defining R03 that was calculated using DH convention-FK
jj = 1; %random parameter for loop
 for ii = 1:4
     sol_i = AllSol_1to3{ii};
     q1 = sol_i(1);
     q2 = sol_i(2);
     q3 = sol_i(3);

    T01=[cos(q1) 0 -sin(q1) 0;
        sin(q1) 0 cos(q1) 0;
        0 -1 0 d1;
        0 0 0 1];
    T12=[cos(q2-pi/2) -sin(q2-pi/2) 0 (a2*cos(q2-pi/2));
        sin(q2-pi/2) cos(q2-pi/2) 0 (a2*sin(q2-pi/2));
        0 0 1 0;
        0 0 0 1];
    T23=[cos(q3+pi/2) -sin(q3+pi/2) 0 (a3*cos(q3+pi/2));
        sin(q3+pi/2) cos(q3+pi/2) 0 (a3*sin(q3+pi/2));
        0 0 1 0;
        0 0 0 1];

    T03=T01*T12*T23;
    R03=T03(1:3,1:3);

    %extracting R from T0e

    R=T0e(1:3,1:3);


    %finding R35=R3e using the eq. R=R03*R3e ; also R03 inverse = transpose of R03

    R35= R03' * R;

    %{
    using forward kinematics on the wrist we know that

    R35 = [sin(theta4)*cos(theta5), -sin(theta4)*sin(theta5), cos(theta4);
           -cos(theta4)*cos(theta5), cos(theta4)*sin(theta5), sin(theta4);
           -sin(theta5), -cos(theta5), 0];

    we will equate this with R35 found above to find theta 4 and theta 5 
    %}

    q4=atan2(R35(2,3),R35(1,3));
    q5=atan2(-R35(3,1),-R35(3,2));
    
    qq = [q1 q2 q3 q4 q5];
    qq = wrapToPi(qq);
        %---------------
        % add to limit
        % ~sum(~[1 1 1]) == 1 ; ~sum(~[1 1 0]) == 0 to determine the logic array is all one
        if ~sum( ~(qq >= lowerLim(1:5)) ) && ~sum( ~(qq <= upperLim(1:5)) )
            q(jj,:) = qq;
            jj = jj + 1;
        else 
            continue
        end
 end               
 %if q does not change, clear it-
        if ~sum(~(q == [0 0 0 0 0]))
            q = [];
            
        end
        

 end
function isPos = checkfeasibility(T)
o = T(1:3,4);
theta1 = mod(atan2(o(2),o(1)),2*pi);
y0_theta = [-sin(theta1);cos(theta1);0];
x0_theta = [cos(theta1);sin(theta1);0];
endaxis_z = T(1:3,3);
% z3.z5 should always be 0 due to the lack of yaw motion; also z3 is in the same
% direction as y0 rotated by theta1 about z0.
a=y0_theta;
b=endaxis_z;
p=round(atan2(norm(cross(a,b)), dot(a,b)),2);

if abs(p) ~= round((pi/2),2)
    isPos = 0;  
%     disp('isPos = False');
else
    isPos = 1;    
%     disp('isPos = True');
end

end
function T2 = corrected(T)
o = T(1:3,4);
theta1 = atan2(o(2),o(1));
y0_theta = [-sin(theta1);cos(theta1);0];
x0_theta = [cos(theta1);sin(theta1);0];
endaxis_z = T(1:3,3);
% z3*z5 should always be 0, and z3 is y0 rotate theta1 according to z0.
    % project endaxis_z into plane YoZ.
    endaxis_z2 = project(endaxis_z,y0_theta);      %defined in line 242-245
    endaxis_z2 = endaxis_z2/norm(endaxis_z2);
    % use axis/angle representation
    axis = cross(endaxis_z,endaxis_z2);
    axis = axis/norm(axis);
    angle = acos(endaxis_z'*endaxis_z2);
    R_corr = ax_an_R(axis,angle);                  %defined in line 247-254
    R_end = R_corr*T(1:3,1:3);
    T2 = [R_end o;0 0 0 1]; 
%     disp('T correct is');
%     disp(T2);
end
% theta1 = atan2(o(2),o(1));
% y0_theta = [-sin(theta1);cos(theta1);0];
% endaxis_z = T0e(1:3,3);
% disp('check:');disp(abs(y0_theta'*endaxis_z))
function b = project(a,n)
%project vector a into plane whose norm vector is n
b = a - (a'*n)/(norm(n)^2)*n;
end

function R = ax_an_R(k,th)
%rotate theta according to k
kx = k(1); ky = k(2); kz = k(3);
vt = 1 - cos(th);
R = [kx^2*vt+cos(th),    kx*ky*vt-kz*sin(th), kx*kz*vt+ky*sin(th);
    kx*ky*vt+kz*sin(th), ky^2*vt+cos(th),     ky*kz*vt-kx*sin(th);
    kx*ky*vt-ky*sin(th), ky*kz*vt+kx*sin(th), kz^2*vt+cos(th)];
end
