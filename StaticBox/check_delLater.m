% % 
function final(color)
    color = 'blue';
  %  global lynx % necessary to use ArmController inside a function
   % lynx = ArmController(color);

    pause(1)
  
    dynamic(color);
    %static(color);
    % interact with simulator, such as...
    
end

function [sdone] = static(color)
    
    global lynx
    lynx = ArmController(color);
    % get state of your robot
    pause(1)
    [q,qd]  = lynx.get_state()

    % get state of scoreable objects
   [name,pose,twist] = lynx.get_object_state();
%     name
%     celldisp(pose)
%     celldisp(twist)

%     for i=1:13
%         if (sum(twist{i}) == 0)
%             st(i,:) = 1;
%         end
%     end
%     [n ~]=find(st > 0);
%     
    %find euclidean distance between goal and closes 4 objects
    [q0, ~] = lynx.get_state();
    goalb = [ 30; 470; 40];
    goalr = [-30; -470; 40];
    %goalTrans= [cos(q0(1)), -sin(q0(1)), 0, 200 ; sin(q0(1)), cos(q0(10)), 0, 200; 0, 0, 1, 0; 0, 0, 0, 1] * [1, 0, 0, 170; 0, 1, 0, -270; 0, 0, 1, 40; 0, 0, 0, 1];
    %for blue only
    [a ~]=size(name);
    
    dist = zeros(a,1);
    for i=1:a
        dist(i,1) = norm(pose{i}(1:3,4) - goalb);
    end
    dist
    
    priority=zeros(4,1);j=1;i=1;
    while sum(find(priority == 0) > 0) 
        if ((dist(i,1) == min(dist(:,1))) & j < 5)
            priority(j,:)=i;
            j=j+1;
            dist(i,1) = 1000;
            i=1;
        else
            i=i+1;
        end
    end
    priority
    
    for i=1:4
        static.name{i,1} = name{priority(i,:)};
        static.pose{i,1} = pose{priority(i,:)};
    end
    %picking static block
    
    
    %for i=1:4
    static.pose{1}
    %transforming box into robot's frame

    T = [ -1, 0, 0, 200; 0, -1, 0, 200; 0, 0, 1, 0; 0, 0, 0, 1] * [cos(q0(1)), -sin(q0(1)), 0, 0; sin(q0(1)), cos(q0(1)), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1] * static.pose{1};
                                           
    [q, ~] = calculateIK(T)
    q1 = q(1, :);
    q1 = [q1, 30];

    lynx.set_pos(q1);
    [q ~] = lynx.get_state();
    reach = 0;
    
    while (reach == 0)
        [q ~] = lynx.get_state();
        if norm(q - q1) < 0.05
            reach = 1;
        end
    end
    
    q1(1,6) = -15;    
    qGrab = q1;
    lynx.set_pos(qGrab);
    grab = 0;
    
    while (grab == 0)
        [q, ~] = lynx.get_state();
        if norm(q - qGrab) < 0.05
            grab = 1;
        end
    end
    
    Tpick = T * [1, 0, 0, 0; 0, 1, 0, 0; 0, 0, 1, +40; 0, 0, 0, 1];
    [qpick, ~] = calculateIK(Tpick)
    qpick(1,6) = -15;
    lynx.set_pos(qpick);
    pick = 0;
    
    while (pick == 0)
        [q, ~] = lynx.get_state();
        if norm(q - qpick) < 0.05
            pick = 1;
        end
    end
    
    move  = 0;
    Tplace = goalTrans
    [qPlace, ~] = calculateIK(goalTrans);
    qPlace(1,6) = -15;
    lynx.set_pos(qPlace)
    
    while (move == 0)
        [q, ~] = lynx.get_state();
        if norm(q - qPlace) < 0.05
            move = 1;
        end
    end
    
    place = 0;
    TDrop = goalTrans * [1, 0, 0, 0; 0, 1, 0, -40; 0, 0, 1, 0; 0, 0, 0, 1];
    [qDrop, ~] = calculateIK(Tdrop);
    qDrop(6,0) = 30;
    lynx.set_pos(qDrop)
    drop = 0;
    
    while (drop == 0)
        [q, ~] = lynx.get_pos();
        if norm(q - qDrop) < -0.05
            drop = 1;
        end
    end
        
    %move back up
    lynx.set_pos(qPlace);
    
    disp(" Placed static object ");
    % Tranformation matrix at goal
    
    

   % lynx.set_pos(q);
    
%%% manual testss end
 % %   get state of your opponent's robot
 %   [q,qd]  = lynx.get_opponent_state()
end
function [done] = dynamic(color)
% % dynamic --> 
    
     global lynx
     lynx = ArmController(color);
     pause(1)
     
    [name,pose,twist] = lynx.get_object_state();
    [num ~] = size(name);
    dynamic = zeros(num,1);j=1;
%     celldisp(twist)
    
    for i=1:num
        if (sum(twist{i}) ~= 0)
            dynamic(j,:) = i;
            j=j+1;
        end
    end
    
    dynamic = dynamic(dynamic~=0);
    
    [num2 ~] = size(dynamic);
    
    %for i=1:num2
    a2 = dynamic(1); % dynamic{i} in loop
    dis = pose{a2}(1:2, 4);
    
    x=norm(dis)*cos(pi/8);
    T= [ cos(pi/8), -sin(pi/8), 0, (200 - x); sin( pi/8), cos(pi/8), 0, (200 - x); 0, 0, 1, (80); 0, 0, 0, 1];     
    
    % % move
    [qMove, ~] = calculateIK(T);
    lynx.set_pos(q);
    move = 0;
    
    while (move == 0)
        [q, ~] = lynx.get_pos();
        if norm(q - qMove) < 0.05
            move = 1;
        end
    end
    
    %Stage for block
    [qStage, ~] = [qMove(1:4), -pi/2, +30];
    lynx.set_pos(qStage);
    stage = 0;
    
    while (stage == 0)
        [q, ~] = lynx.get_pos();
        if norm(q - qStage) < 0.05
            stage = 1;
        end
    end
    
    % wait
    wait = 0;
    
    while (wait == 0)
        [q, ~] = lynx.get_pos();
        [~, Position] = calculateFK(q);
        Position = Position(6,:); %% Joint 6 location
        [~, pose, ~] = lynx.get_object_state();
        location = pose{a2}(1:3,4);
        
        if norm(location - position) < 0.05
            wait = 1;
            q2 = [q(1:4), 0, +30];
            lynx.set_pos(q2);
            orient = 0;
            
            while (orient < 2)
                [qCheck, ~] = lynx.get_pos();
                if norm(q2 - qCheck) < 0.05
                    orient = 1;
                    q3 = [q2(1:5), -15];
                    [qCheck2, ~] = lynx.get_pos();
                    if norm(q3 - qCheck2) == 0
                        oreint = 2;
                    end
                end
            end
        end
    end
    
     move  = 0;
    Tplace = goalTrans
    [qPlace, ~] = calculateIK(goalTrans);
    qPlace(1,6) = -15;
    lynx.set_pos(qPlace)
    
    while (move == 0)
        [q, ~] = lynx.get_state();
        if norm(q - qPlace) < 0.05
            move = 1;
        end
    end
    
    place = 0;
    TDrop = goalTrans * [1, 0, 0, 0; 0, 1, 0, -40; 0, 0, 1, 0; 0, 0, 0, 1];
    [qDrop, ~] = calculateIK(TDrop);
    qDrop(6,0) = 30;
    lynx.set_pos(qDrop)
    drop = 0;
    
    while (drop == 0)
        [q, ~] = lynx.get_pos();
        if norm(q - qDrop) < -0.05
            drop = 1;
        end
    end
    
    %move back up
    lynx.set_pos(qPlace);
    disp(" Placed Dynamic object ")
    done = 1;
end
    % % DYNAMIC END
    
   

% % IK STARTS
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
    q = [];fprintf('\n');disp('Point is outside reachable workspace');
    isPos = 0; disp('isPos = False');
    return 
end

%determine the feasibility of orientation
%Note that theta1 is fixed by the position, only theta4 and theta5 dedicate to the end pose orientation.
fprintf('\n');
disp('---checking feasibility:---');

isPos = checkfeasibility(T0e);          %defined in line 200-218


if ~isPos
    disp('-- After correction --')
    T0e = corrected(T0e);               %defined in line 219-254
end
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
           
            fprintf('\n');
            fprintf('\n');
            disp(' ALL solutions are out of the joint limits ! ! ')
            fprintf('\n');
            fprintf('\n');
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
    isPos = 0;    disp('isPos = False');
else
    isPos = 1;    disp('isPos = True');
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
    disp('T correct is');
    disp(T2);
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


%%% IK ENDS

%%%FK STARTS
function [jointPositions,T0e] = calculateFK(q)
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

%getting all link transformations by calling mat function per DH convention
T01=mat(q(1),-pi/2,0,L1);
T12=mat(q(2)-pi/2,0,L2,0);
T23=mat(q(3)+pi/2,0,L3,0);
T34=mat(q(4)-pi/2,-pi/2,0,0);
T45=mat(q(5),0,0,L4+L5);

%generate the T of each coordinate
T02=T01*T12;
T03=T02*T23;
T04=T03*T34;
T05=T04*T45;
%T05 is T0e
T0e=T05

%get all the origin of frames from their transformation matrix
%Note that joint 4 is not the origin of coordinate{4}
joint2=T01(1:3,4)';
joint3=T02(1:3,4)';
joint4=T03(1:3,4)';
joint6=T05(1:3,4)';
joint5=1/2*(joint4+joint6);  %Obviously, joint5 is midpoint of joint 4,6
%form the jointPosition
jointPositions=[0,0,0;joint2;joint3;joint4;joint5;joint6];


%Defining function to calculate homogeneous matrix
function [tr]=mat(t,al,a,d)
    %rotating about z, Rz
    function [x]=rotzx(t)
    x=[cos(t) -sin(t) 0 0;sin(t) cos(t) 0 0;0 0 1 0;0 0 0 1];
    end
%rotating about x, Rx
    function [y]=rotxx(al)
    y=[1 0 0 0;0 cos(al) -sin(al) 0;0 sin(al) cos(al) 0;0 0 0 1];
    end
%translation about z, TRz
    function [z]=tran1(d)
    z=[1 0 0 0;0 1 0 0;0 0 1 d;0 0 0 1];
    end
%translation about x, TRx
    function [u]=tran2(a)
    u=[1 0 0 a;0 1 0 0;0 0 1 0;0 0 0 1];
    end
%homogenrous matrix = Rz*TRz*TRx*Rx sequence
tr=rotzx(t)*tran1(d)*tran2(a)*rotxx(al);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end

% % FK ENDS

% % %b
%     
% % p=[1,2,3;
% %     4,5,6;
% %     7,8,9;
% %     9,8,7;
% %     6,5,4;
% %     3,0,1];
% % 
% % length(p);
% %zeros(size(p,1),1)
% %size(p,1) %returns length of the first column/dimension of p/array
% % max(p,[],1)
% % max(p,[],2)
% % vecnorm([2,0,5],2,2)
% % sign(-1*-8)
% % %  q=[0 0 -pi/2 0 0 0];
% %  v =[200;100;NaN];
% %  omega =[0;0;3];
% %  qdot=IK_velocity(q,v,omega,joint)
% % dq = [1 1 1 1 1 1]; 
% % [v w] = FK_velocity(q,dq,joint)
% 
% 
% % 
% 
% % q=[0 0 -pi/2 0 0 0];
% % v =[10;1.5;200];
% % omega =[1;0;0];
% % qdot=IK_velocity(q,v,omega,joint)
% 
% %dq = [1 0 1 0 1 0];
% 
% %[v w] = FK_velocity(q,dq,joint)
% 
% % %  %v =[-127.6625;86.8625;134.4625];
% % % % v=[-127.65;NaN;NaN];
% % % % omega =[1.2000;-1.1000;0.5000];
% % % v =[127.6625;110.6625;-161.6625];
% % % omega =[0.5000;NaN;0.5000];
% % v =[10;1.5;200];
% % omega =[1;0;0];
% % qdot=IK_velocity(q,v,omega,joint)
% 
% % a=find(isnan(v))
% % if sum(a)>1
% %     disp('Yes')
% % else
% %     disp('No')
% % end
% % % 
%  % % % 
% % 
% % Si=[127;110;-161;NaN;NaN;0]
% % delrow=zeros(6,1);j=1;
% % 
% % for i=1:6
% %     if isnan(Si(i,:))
% %         j=i;
% %         delrow(j,1)=1;
% %     end
% % end
% % delrow
% % [c ~]=find(delrow>0)
% % Si(c,:)=[];
% % Si
% 
% 
% %nodes=ones(3,5);
% %[rows, cols]=size(nodes);
% %a=rows;
% %dist=[];
% %dist=dist12
% 
% %a=[]
% %for i=1:10
%  %   a=[a;i];
% %end
% %a=[];
% %for i=10:-1:1
%  %   a=[a;i];
% %end
% %a
% 
% %path=[20];
% %a=[1:5,14:18]';
% %b=[11:20]';
% %[j ~]=size(b);
% %{
% for i=10:-1:1
% %    path=[a(j);path];
%  %   j=(b == a(j));
%   
%     path=[a(j);path];
%     c=a(j);
%     z=find(b=='c');
%     j=z;
%     
% 
% end
% path
% %}
% 
% %a=1:10;
% %b=11:20;
% %x=find(b==17);
% %y=find(a==a(x));
% %a(y-2)
% 
% %{
% a=[1,2,3,4;5,6,7,8;9,8,7,6];
% b=[5,6,7,8];
% if false(ismember(b,a))
%     disp('false')
% else
%     disp('true')
% end
% %}