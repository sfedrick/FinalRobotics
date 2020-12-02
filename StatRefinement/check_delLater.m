
    addpath('../Main/Core')
    color = 'blue';
   % global lynx % necessary to use ArmController inside a function
   % lynx = ArmController(color);

   % pause(1)
  
    %dynamic(color);
    static(color);
    % interact with simulator, such as...
    

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
    lynx.set_pos(qMove);
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