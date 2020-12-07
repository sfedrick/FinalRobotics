function [close] = rotationdriver(lynx,color,r)
    %% Simulation Parameters

%start = [.9, 0, 1, -1, -pi/2,30];
%start=[0.75, 0.1, 0.9, -1, -pi/2,30];
%start=[0.75, 0.1, 0.7, -0.8, -pi/2,30
%r = calculateRadiusForEndEff(lynx,color);
pause(0.1);
[start,~]=findperfect(r,-5) ; 
move(start,lynx);
[closeup,~] = InUrFace(lynx,color,25,0,3,0,1);

<<<<<<< HEAD
    [q,qd]  = lynx.get_state()
    [jointvel,configs] = MoveInCircle(r,1,100,q,0);
    qold=q;
=======
% ToleranceMovement(lynx,color,start,0.1,3,1);
close=false;
[q,~]  = lynx.get_state();    
[~,RoboPose] = calculateFK(q);
collided=lynx.is_collided();
EndLocation=RoboPose(1:3,4); 
>>>>>>> 7a881bd25aec58985899ed8378c6b9ab3c80731c

if(EndLocation(3)>75)
    disp("something was below me");
   blockingpath=true;
elseif(closeup)
     disp("something was below me and in my face");
    close=true;
    blockingpath=true;
else
    blockingpath=false;
end

[jointvel,configs] = MoveInCircle(r,1,100,q,0);
qold=q;
L=length(jointvel(:,1));
range=50;
direction=0.8;
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
            [lucky,luckyboxes] = InUrFace(lynx,color,30,0.5,3,0,1);
            %[sanity,~] = InUrFace(lynx,color,90,0,3);
            [Stop] = QuarterGrab(q,qold,0.01);
            if(WithInFace)
                disp("was in face so broke out of path")
                lynx.set_vel([0,0,0,0,0,100]);
                close=false;
                break;
            end
%             if(Stop)
%                 lynx.set_vel([0,0,0,0,0,100]);
%                 close=true;
%                  break;
%             end
            if(lucky)
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
       
<<<<<<< HEAD
        dq = jointvel(target_index, :);
        q = configs(target_index,:);
        disp("Goal:")
        disp(dq)
        dq(6)=100;
        lynx.command(q)
%          %[breakme]=ToleranceVelocity(lynx,dq,0.2,100);
        ToleranceMovement(lynx,q,0.1,1000);
        lynx.set_vel(dq);
        [WithInFace,BoxesInUrFace] = InUrFace(lynx,color,50,0.9,3);
        
        [Stop] = QuarterGrab(q,qold,0.01);
        if(Stop||WithInFace)
            lynx.set_vel([0,0,0,0,0,100]);
            break;
        end
        [pos, ~] = lynx.get_state();
        qold=pos;
   end
   lynx.set_vel([0,0,0,0,0,100]);
%    lynx.set_vel([0,0,0,0,0,100])
%    close=false;
%    wait=WithInFace;
%    counter=0;
%    olddistancevec=[];
%    distancevec=[];
%    dv=inf;
%    flipper=1;
%    while(~close && (wait || WithInFace))
%        counter=counter+1;
%        if(mod(counter,20)==0)
%            flipper=flipper+1;
%            if(mod(flipper,2)==0)
%                 olddistancevec=distancevec;
%            else
%                dv=norm(olddistancevec-distancevec);
%            end
%        end
%        
%        [WithInFace,trash] = InUrFace(lynx,color,30,0.85,3);
%       [close,wait,BoxesInUrFace,speed,distancevec]=grip(lynx,WithInFace,BoxesInUrFace,color,20,0.5,50);
%       if(dv<0.01 && norm(distancevec)<30)
%           close=true;
%       end
%    end
%  
% [ender,~]= lynx.get_state();
% ender(6)=-100;
% if(close)
%    ToleranceMovement(lynx,ender,0.1,1000);
% end
% [pos, ~] = lynx.get_state(); 
% if(pos(6)<2)
%     close=false;
% end
=======
       wait=WithInFace;
       if(~wait)
           disp("wasn't in face")
       end
       counter=0;
       
       movebuffer=10;
       ramp=1;
       dramp=0.1;
       firsttime=true;
       [~,distancevec,originaltarget]=grip(lynx,color,BoxesInUrFace,100);
       range=norm(distancevec);
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
            direction=direction-dramp;
           [q,~]=lynx.get_state();
         [~,RoboPose] = calculateFK(q);
           EndLocation=RoboPose(1:3,4);

           if(EndLocation(3)>70)
               [q,isPos] = goDown(q,-10);
               lynx.set_pos(q);
           end

           [WithInFace,BoxesInUrFace] = InUrFace(lynx,color,range,direction,3,0,1);
           [velocityFace,velFaces] = InUrFace(lynx,color,range,direction,3,1,1);
           [left,~] = InUrFace(lynx,color,20,0.8,3,1,1);
           [right,~] = InUrFace(lynx,color,20,0.8,-3,1,1);
          [~,distancevec,newtarget]=grip(lynx,color,BoxesInUrFace,range-movebuffer);
          %range=norm(distancevec); figure out how to implement this 
          [closeup,~] = InUrFace(lynx,color,25,0,3,0,1);
          if(range<30|| closeup)
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
               [~,distancevec,originaltarget]=grip(lynx,color,BoxesInUrFace,range-movebuffer);
          elif(~WithInFace)
              disp("Nothing happend so I broke out");
              disp(range)
              disp(direction)
              disp(closeup)
               wait=false;
          end

         rosPause(0.1);
       end
>>>>>>> 7a881bd25aec58985899ed8378c6b9ab3c80731c

        [ender,dq]= lynx.get_state();
        dender(6)=-5;
        ender(6)=-5; 
        lynx.set_pos(ender);
        lynx.set_vel(dender);
        rosPause(0.5);
       disp("result");

        
        [pos, ~] = lynx.get_state(); 
        if(pos(6)<2)
            close=false;
        else
             close=true;
        end
end
    
end
