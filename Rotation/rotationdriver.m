function [close] = rotationdriver(lynx,color,r)
    %% Simulation Parameters
close=false;
    %start = [.9, 0, 1, -1, -pi/2,30];
    %start=[0.75, 0.1, 0.9, -1, -pi/2,30];
    %start=[0.75, 0.1, 0.7, -0.8, -pi/2,30
    %r = calculateRadiusForEndEff(lynx,color);
    pause(0.1);
    [start,~]=findperfect(r,-15) ; 
    lynx.set_pos(start);
    ToleranceMovement(lynx,color,start,0.1,10,1);
    close=false;
    [blockingpath,names]=InUrFace(lynx,color,100,0.5,-2,0,0);
    if(blockingpath)
        grip(lynx,color,names,20);
        [ender,~]= lynx.get_state();
        ender(6)=-5;
        lynx.set_pos(ender);
        ToleranceMovement(lynx,color,ender,0.1,10,1);
        [pos, ~] = lynx.get_state(); 
        if(pos(6)<2)
            close=false;
        end
    end

    [q,qd]  = lynx.get_state();
    [jointvel,configs] = MoveInCircle(r,1,100,q,0);
    qold=q;
L=length(jointvel(:,1));
range=100;
direction=0.9;
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
%                 lynx.set_pos(q);
%                 ToleranceMovement(lynx,color,q,0.3,1,1)
%            
%            else
%                lynx.command(q);
%                ToleranceMovement(lynx,color,q,0.3,1,0.1)
%            
%            end
                
           
           
           [WithInFace,BoxesInUrFace] = InUrFace(lynx,color,range,direction,3,1,1);
            [lucky,luckyboxes] = InUrFace(lynx,color,20,0.9,3,0,1);
            %[sanity,~] = InUrFace(lynx,color,90,0,3);
            [Stop] = QuarterGrab(q,qold,0.01);
            if(WithInFace)
                lynx.set_vel([0,0,0,0,0,100]);
                close=false;
                break;
            end
            if(Stop)
                lynx.set_vel([0,0,0,0,0,100]);
                close=true;
                 break;
            end
            if(lucky)
                lynx.set_vel([0,0,0,0,0,100]);
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

       
       wait=WithInFace;
       counter=0;
       
       movebuffer=10;
       ramp=0.1;
       firsttime=true;
       [close,distancevec,originaltarget]=grip(lynx,color,BoxesInUrFace,100);
       range=norm(distancevec);
       while(~close &&  wait)
           counter=counter+1;
    %        if(mod(counter,50)==0)
    %            flipper=flipper+1;
    %            if(mod(flipper,2)==0)
    %                 olddistancevec=distancevec;
    %            else
    %                dv=norm(olddistancevec-distancevec);
    %            end
    %        end

            range=range-ramp*counter;
            direction=direction-0.01*ramp*counter;
           [q,~]=lynx.get_state();
         [~,RoboPose] = calculateFK(q);
           EndLocation=RoboPose(1:3,4);

           if(EndLocation(3)>70)
               [q,isPos] = goDown(q,-10);
               lynx.set_pos(q);
               break;
           end

           [WithInFace,BoxesInUrFace] = InUrFace(lynx,color,range,direction,3,0,1);
           [velocityFace,velFaces] = InUrFace(lynx,color,range,direction,3,1,1);
          [close,distancevec,newtarget]=grip(lynx,color,BoxesInUrFace,range-movebuffer);
          %range=norm(distancevec); figure out how to implement this 
          if(range<30)
              disp("counter broke me out")
              disp(range)
              disp(direction)
              close=true;
              break;
          elseif(velocityFace)
              close=grip(lynx,color,velFaces,20);
              wait=true;
          elseif(~WithInFace)
              firsttime=false;
              %[rightBehind,BoxesInUrFace] = InUrFace(lynx,color,10,0,-3,0,1);
              [closeup,~] = InUrFace(lynx,color,30,0,3,0,1);
              if(closeup)
                    close=true;
              else
              end
           elseif(strcmp(originaltarget,newtarget))
              wait=true;
           elseif(WithInFace && ~strcmp(originaltarget,newtarget) && velocityFace)
             [close,distancevec,originaltarget]=grip(lynx,color,velFaces,range);
           elseif(WithInFace && ~strcmp(originaltarget,newtarget))
               [close,distancevec,originaltarget]=grip(lynx,color,BoxesInUrFace,range-movebuffer);
           else
               wait=false;
          end

         rosPause(0.1);
       end

        [ender,dq]= lynx.get_state();
        dender(6)=-5;
        ender(6)=-5;
        if(close)
            
            lynx.set_pos(ender);
            lynx.set_vel(dender);
            rosPause(0.5);
           disp("result");
         
        end
        [pos, ~] = lynx.get_state(); 
        if(pos(6)<2)
            close=false;
        end
end
    
end
