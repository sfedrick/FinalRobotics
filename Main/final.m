function final(color)
    
    
    global lynx % necessary to use ArmController inside a function
    lynx = ArmController(color);

    pause(2)
    c=[190,-176,29];
    ee1=[318,-59,35];
    ee=[190,-175,-73];
    
    %dis=norm(c-ee)
    [q w e]=lynx.get_object_state();
    home = [0 0 0 0 0 0];
    % interact with simulator, such as...
    q1 = [-0.747,0.587,-0.281,1.272,-0.586,20];
%     qRed = [-0.747,0.587,-0.281,1.272,-0.586,20];
% % % % %     [a b]= InUrFace(lynx,color,100,-1,3)
% % % % %     [c d] = InUrFace(lynx,color,60,0,1)
    %     fprintf('\n here \n');
    lynx.command(q1);
%     disp(' there')
   % pause(2)
    % get state of your robot
   % [q,~]  = lynx.get_state()
    % [a b] = calculateFK(qRed)
    
%     T=[0, -1, 0, 198; -1, 0, 0, -233; 0, 0, -1, 68; 0, 0, 0, 1];
%     [q2 ~] = calculateIK(T)
%     
%     [c d] = calculateFK(q2)
% 
%     [name,pose,twist] = lynx.get_object_state();
% 
% 
%     [q,qd]  = lynx.get_state()
    % % %   get state of your opponent's robot 
    %[q,qd]  = lynx.get_opponent_state()



end
function [WithInFace,BoxesInUrFace] = InUrFace(lynx,color,Range,Direction,Axis)
% This function alerts you when the robot has a box in its face
%lynx is the lynx variable from lynx=ArmController(color)
%color is a character string
%Range is the detection radius of the robot a typical value is 100
%Direction is a number from -1 to 1 that determines the acceptable direction 
%normally direction >0 means boxes that are in front of robot
%direction<0 are boxes that are behind the robot end effector a typical value 
% is Direction=.70;
%Axis determines the axis of the end effector you are concerned with 
%1= x axis 2=y axis 3=zaxis you typically only need 1 or 3 
WithInFace=false;

%[q,~]=lynx.get_state()
q2=[-0.186;1.291;-1.219;1.547;-0.157;20.0040];
q1=[-0.747,0.587,-0.281,1.272,-0.586,20];
q=q2;
[RoboLocation,RoboPose] = calculateFK(q)
%EndLocation=RoboPose(1:3,4)
EndLocation = RoboLocation(6,:)';
RoboFinger=RoboPose(1:3,Axis)
[name,pose,twist]=lynx.get_object_state();

N=length(pose);
%transform 
if(strcmp(color,'red'))
    H=[1,0,0,200;
       0,1,0,200;
       0,0,1,0;
       0,0,0,1];
elseif(strcmp(color,'blue'))
   H=[-1,0,0,-200;
       0,-1,0,-200;
       0,0,1,0;
       0,0,0,1]; 
end
BoxesInUrFace=[];
    for i=1:N
        %create vector from end effector to center of each box 
        %perform dot product to determine direction
        %use norm to determine distance from end effector 
        currentbox=pose{i};
        name{i}
        BoxFrame=H*currentbox;
        BoxLocation=BoxFrame(1:3,4)
        VecToBox=BoxLocation-EndLocation
        normVecTOBox=norm(VecToBox)
        UnitVecToBox=VecToBox/normVecTOBox;
        dotProduct=UnitVecToBox'*RoboFinger
        if(normVecTOBox<Range && dotProduct>Direction)
            BoxesInUrFace=[BoxesInUrFace;name{i}];
            WithInFace=true;
        end
    end 


end
