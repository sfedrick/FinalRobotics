function [breakme] = ToleranceMovement(lynx,desiredpos,error,base)
        reached_target = false;
        i=0;
        
        breakme=false;
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
            end
            pause(0.1)
            % End of student code
        end
   
end
%the one below is the time version it doesn't work yet

% function [breakme] = ToleranceMovement(lynx,desiredpos,error,base)
%         reached_target = false;
%         breakme=false;
%         OGtime=rostime("now");
%         nowtime=rostime("now");
%         time=OGtime.Nsec-nowtime.Nsec;
%         timelimit=1*10^9;
%         
%         while ~reached_target
%             if(abs(time)>timelimit || time<0)
%                 breakme=true;
%                 break;
%             end
%             % Check if robot is collided then wait
%             nowtime=rostime("now");
%             time=OGtime.Nsec-nowtime.Nsec;
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
%                % reached_target = true;
%                disp("i broke out");
%             end
%             rosPause(0.01);
%             % End of student code
%         end
%    
% end
% 
