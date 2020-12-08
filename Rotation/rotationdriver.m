function [close] = rotationdriver(lynx,color,r)
%% This is the main rotation driver function that works by drawing a circle around the table 
% it works by walking around a table based on a given radius from the
% calculateRadiusForEndEff(lynx, color,axis) function
%it just needs r from this function and the lynx and color function
%it walks along the circle and once an obstacle is in its face it moves the
%end effector towards the block 
%these are key parameters for the function

%this determines how far a block has to be from the robot in order for us 
% to force movement towards the block
%
    movebuffer=10;
    ramp=0.1;
    dramp=0.05;
    range=60;
    direction=0.8;
    leftrightRange=10;
    
[q, ~] = lynx.get_state();
 [q,~] = goDown(q,60);
 lynx.set_pos(q);
 ToleranceMovement(lynx,color,q,0.1,1,0)
 [start,~]=findperfect(r,-10) ; 
lynx.set_pos(start);
ToleranceMovement(lynx,color,start,0.1,3,0)
[closeup,~] = InUrFace(lynx,color,20,0,3,0,0);
[below,blockingboxes] = InUrFace(lynx,color,30,0.8,2,0,0);

% ToleranceMovement(lynx,color,start,0.1,3,1);
close=false;
[q,~]  = lynx.get_state();    
[~,RoboPose] = calculateFK(q);
EndLocation=RoboPose(1:3,4); 

if(EndLocation(3)>75)
    disp("something was below me");
   blockingpath=true;
elseif(closeup)
     disp("something was below me and in my face");
    close=true;
    blockingpath=true;
elseif(below)
     [~,distancevec,originaltarget]=grip(lynx,color,blockingboxes,range+movebuffer);
else
    blockingpath=false;
end

[jointvel,configs] = MoveInCircle(r,1,50,q,0);
qold=q;
L=length(jointvel(:,1));
if(~blockingpath)
       for target_index = 1:L

            dq = jointvel(target_index, :);
            q = configs(target_index,:);
%             q(5)=-pi/2;
%             q(6)=(30);
%             disp("Goal:")
%             disp(dq)
            dq(5)=0;
            dq(6)=100;
            lynx.set_vel(dq);

           %  [breakme]=ToleranceVelocity(lynx,color,q,0.2,100,0.5);
           
            breakme=JerkMove(lynx,color,q,0.3,1,0.1,2);
%            if(lynx.is_collided())
%                 disp("i collided")
%            end
                
           
           
           [WithInFace,BoxesInUrFace] = InUrFace(lynx,color,range,direction,3,1,1);
           [left,~] = InUrFace(lynx,color,leftrightRange,0.5,1,1,1);
           [right,~] = InUrFace(lynx,color,leftrightRange,0.5,-1,1,1);
           
            [lucky,luckyboxes] = InUrFace(lynx,color,20,0.5,3,0,0);
            %[sanity,~] = InUrFace(lynx,color,90,0,3);
            [Stop] = QuarterGrab(q,qold,0.01);
            if(WithInFace)
                disp("was in face so broke out of path")
                lynx.set_vel([0,0,0,0,0,100]);
                close=false;
                break;
%             elseif(~left && ~right)
%                  disp("a block was to the left or right of me")
%                 lynx.set_vel([0,0,0,0,0,100]);
%                 close=false;
%                 break;
            elseif(lucky)
                lynx.set_vel([0,0,0,0,0,100]);
                disp("I got lucky");
                close=true;
                break;
            end
    %         if(~sanity && ((target_index/L)>0.3))
    %             lynx.set_vel([0,0,0,0,0,100]);
    %             break;
    %         end
            [pos, ~] = lynx.get_state();
            qold=pos;
       end
disp("finished")
       
       wait=WithInFace;
       if(~wait)
           disp("wasn't in face")
       end
       counter=0;
       
        direction=0.1;
       firsttime=true;
       [~,distancevec,originaltarget]=grip(lynx,color,BoxesInUrFace,100);
       while(~close &&  wait)
            lynx.set_vel([0,0,0,0,0,100]);
           counter=counter+1;
    %        if(mod(counter,50)==0)
    %            flipper=flipper+1;
    %            if(mod(flipper,2)==0)
    %                 olddistancevec=distancevec;
    %            else
    %                dv=norm(olddistancevec-distancevec);
    %            end
    %        end

            range=range-ramp;
            direction=abs(direction-dramp);
           
           [q,~]=lynx.get_state();
         [~,RoboPose] = calculateFK(q);
           EndLocation=RoboPose(1:3,4);

           if(EndLocation(3)>70)
               [q,~] = goDown(q,60);
               lynx.set_pos(q);
           end

           [WithInFace,BoxesInUrFace] = InUrFace(lynx,color,range,direction,3,0,1);
           [velocityFace,velFaces] = InUrFace(lynx,color,range,direction,3,1,1);
           [left,~] = InUrFaceNoSafe(lynx,color,leftrightRange,0.9,1,1,1,0);
           [right,~] = InUrFaceNoSafe(lynx,color,leftrightRange,0.9,-1,1,1,0);
          [~,distancevec,newtarget]=grip(lynx,color,BoxesInUrFace,inf);
%           if(norm(distancevec)~=inf)
%             range=norm(distancevec)+movebuffer; 
%           end
          [closeup,~] = InUrFace(lynx,color,15,0,3,0,1);
          if(range<25|| closeup)
              disp("counter broke me out")
              disp(range)
              disp(direction)
              disp(closeup)
              break;
          elseif(velocityFace && ~(left || right))
              [~,distancevec,newtarget]=grip(lynx,color,velFaces,20);
              wait=true;
           elseif(strcmp(originaltarget,newtarget))
              wait=true;
           elseif(WithInFace && ~strcmp(originaltarget,newtarget) && velocityFace)
             [~,distancevec,originaltarget]=grip(lynx,color,velFaces,range);
           elseif(WithInFace && ~strcmp(originaltarget,newtarget))
               [~,distancevec,originaltarget]=grip(lynx,color,BoxesInUrFace,range+movebuffer);
          elseif(~WithInFace && strcmp("",newtarget))
              disp("Nothing happend so I broke out");
              disp(range)
              disp(direction)
              disp(closeup)
               wait=false;
          end

         rosPause(0.1);
       end     
end

        [ender,~]= lynx.get_state();
        ender(6)=-5; 
         %lynx.set_vel([0,0,0,0,0,-5]);
        lynx.set_pos(ender);
        rosPause(0.5);
        [pos, ~] = lynx.get_state(); 
        if(pos(6)<2)
            close=false;
        else
            close=true;
        end
    
end
