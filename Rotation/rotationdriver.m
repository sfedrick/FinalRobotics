function [close] = rotationdriver(lynx,color,r)
    %% Simulation Parameters

    %start = [.9, 0, 1, -1, -pi/2,30];
    %start=[0.75, 0.1, 0.9, -1, -pi/2,30];
    %start=[0.75, 0.1, 0.7, -0.8, -pi/2,30
    %r = calculateRadiusForEndEff(lynx,color);
  [start,~]=findperfect(r,-10);  
    lynx.set_pos(start);
    ToleranceMovement(lynx,start,0.1,1000);

    [q,qd]  = lynx.get_state()
    [jointvel,configs] = MoveInCircle(r,1,100,q,0);
    qold=q;

   for target_index = 1:length(jointvel(:,1))
       
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

    
end
