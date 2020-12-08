
function [breakme] = ToleranceMovement(lynx,color,desiredpos,error,timelimit,scooping)
        reached_target = false;
        i=0;
        
        breakme=false;
       t0=RossyTime();
       time=0;
        while (~reached_target && time<timelimit)
            time=RossyTime()-t0; 
            i=i+1;
            
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
            
            [lucky,luckyboxes] = InUrFace(lynx,color,20,0.9,3,0,1);
            if(lucky && scooping)
                breakme=true;
                break;
            end
        end
        if(time>timelimit && ~scooping)
            breakme=true;
        end
end
%ros time sucks
%function [breakme,dt] = ToleranceMovement(lynx,desiredpos,error,base)
%         reached_target = false;
%         breakme=false;
%         OGtime=RossyTime();
%         nowtime=RossyTime();
%         time=nowtime-OGtime;
%         
%         dt=0.01;
%         posDiff=inf;
%         
%         while ~reached_target
%             timelimit=posDiff*10;
%             if(time>timelimit)
%                 breakme=true;
%                 break;
%             end
%             % Check if robot is collided then wait
%             nowtime=RossyTime();
%             time=nowtime-OGtime;
%             
%             %collision = collision | lynx.is_collided();
%             % Add Student code here to decide if controller should send next
%             % target or continue to wait. Do NOT add additional pauses to control
%             % loop. You will likely want to use lynx.get_state() to decide when to
%             % move to the next target.
%             [pos, vel] = lynx.get_state();
%             %disp(clc"position difference")
%             if(isempty(pos))
%                 dt=dt+0.01;
%             else
%                 posDiff=norm(pos(1:5)-desiredpos(1:5));
%             end
%             
%             if(posDiff<(error))
%                reached_target = true;
%                
%             end
%             rosPause(dt);
%             % End of student code
%         end
%    
% end

