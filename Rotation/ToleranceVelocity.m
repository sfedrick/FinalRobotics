function [breakme] = ToleranceVelocity(lynx,color,desiredpos,error,base,timelimit)
     
        reached_target = false;
        i=0;
        
        breakme=false;
        
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

