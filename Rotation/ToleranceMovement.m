function [reached_target] = ToleranceMovement(lynx,desiredpos,error)
        reached_target = false;
        i=0;
        base=1000;
        Based=false;
        while ~reached_target
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
            elseif(posDiff<(error+(1/base))&& ~Based)
                Based=true;
                base=100;
               % lynx.set_pos(desiredpos);  
            elseif(posDiff<(error+(1/base)))
                reached_target = true;
            end
            pause(0.1)
            % End of student code
        end
   
end

