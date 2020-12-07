<<<<<<< HEAD
% function [breakme] = ToleranceVelocity(lynx,desiredpos,error,base)
%      
%         reached_target = false;
%         i=0;
%         
%         breakme=false;
%         while ~reached_target
%             i=i+1;
%             base=base-i;
%             % Check if robot is collided then wait
%             
%             %collision = collision | lynx.is_collided();
%             % Add Student code here to decide if controller should send next
%             % target or continue to wait. Do NOT add additional pauses to control
%             % loop. You will likely want to use lynx.get_state() to decide when to
%             % move to the next target.
%             [pos, vel] = lynx.get_state();
%             %disp(clc"position difference")
%             
%             posDiff=norm(pos(1:5)-desiredpos(1:5));
%             if(posDiff<(error))
%                 reached_target = true;
%             elseif(posDiff<(error+(1/base)))
%                 reached_target = true;
%             elseif(base<0)
%              breakme=true;
%              reached_target = true;
%             end
%             pause(0.1)
%             % End of student code
%         end
% end

function [breakme] = ToleranceVelocity(lynx,desiredpos,error,base)
=======
function [breakme] = ToleranceVelocity(lynx,color,desiredpos,error,base,timelimit)
     
>>>>>>> 7a881bd25aec58985899ed8378c6b9ab3c80731c
        reached_target = false;
        breakme=false;
<<<<<<< HEAD
        OGtime=RossyTime();
        nowtime=RossyTime();
        time=nowtime-OGtime;
        timelimit=0.1;
        while ~reached_target
            if(abs(time)>timelimit)
                breakme=true;
                break;
            end
            
            time=RossyTime()-OGtime;
            rosPause(0.1);
            % End of student code
        end
end
=======
        
        t0=RossyTime();
        time=0;
        while (~reached_target && time<timelimit)
            time=RossyTime()-t0; 
            i=i+1;
            base=base-i;
            % Check if robot is collided then wait
            
            %collision = collision | lynx.is_collided();
            % Add Student code here to decide if controller should send next
            % target or continue to wait. Do NOT add additional pauses to control
            % loop. You will likely want to use lynx.get_state() to decide when to
            % move to the next target.
            [pos, vel] = lynx.get_state();
            %disp(clc"position difference")
            
            posDiff=norm(pos(1:5)-desiredpos(1:5));
            if(posDiff<(error))
                reached_target = true;
            elseif(posDiff<(error+(1/base)))
                reached_target = true;
            elseif(base<0)
             reached_target = true;
            end
            pause(0.1);
            
            [lucky,luckyboxes] = InUrFace(lynx,color,20,0.9,3,0,0);
            if(lucky)
                breakme=true;
                break;
            end
            % End of student code
        end
    if(time>timelimit)
       breakme=true;
   end
end

>>>>>>> 7a881bd25aec58985899ed8378c6b9ab3c80731c
