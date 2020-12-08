function [WithInFace,BoxesInUrFace] = InUrFace(lynx,color,Range,Direction,Axis,DetectLinVel,ignorez)
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
flip=false;
if(Axis<0)
    Axis=abs(Axis);
    flip=true;
end

[q,~]=lynx.get_state();
[RoboLocation,RoboPose] = calculateFK(q);
EndLocation=RoboPose(1:3,4);
if(ignorez)
    EndLocation(3)=0;
end
RoboFinger=RoboPose(1:3,Axis);
safety=RoboFinger/norm(RoboFinger);
if(ignorez)
safety(3)=0;
end
EndLocation=EndLocation-safety*5;
[name,pose,twist]=filterOutStaticBlocks();
N=length(name);
%transform from robot frame to base frame 
if(strcmp(color,'red'))
    H=[1,0,0,200;
       0,1,0,200;
       0,0,1,0;
       0,0,0,1];
elseif(strcmp(color,'blue'))
   H=[-1,0,0,200;
       0,-1,0,200;
       0,0,1,0;
       0,0,0,1]; 
end
BoxesInUrFace=[];
    for i=1:N
        %create vector from end effector to center of each box 
        %perform dot product to determine direction
        %use norm to determine distance from end effector 
        currentbox=pose{i};
        BoxFrame=H*currentbox;
        BoxLocation=BoxFrame(1:3,4);
        speed=norm(twist{i});
        linvel=twist{i};
        linvel=linvel(1:3);
        linvel=linvel/norm(linvel);
        linvel=H(1:3,1:3)*linvel;
        
        
        
        VecToBox=BoxLocation-EndLocation;
        if(ignorez)
        VecToBox(3)=0;
        end
        if(flip)
            VecToBox=-VecToBox;
        end
        normVecTOBox=norm(VecToBox);
        UnitVecToBox=VecToBox/normVecTOBox;
        dotProduct=UnitVecToBox'*RoboFinger;
        checklinvel=(RoboFinger'*linvel);
        if(~DetectLinVel)
            checklinvel=-inf;
        end
        if((normVecTOBox<Range && dotProduct>Direction)&& checklinvel<-Direction)
            BoxesInUrFace=[BoxesInUrFace;name{i}];
            WithInFace=true;
        end
    end 


end
