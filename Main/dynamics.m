function dynamics(color)
global lynx
%lynx.wait_for_start();
t0=RossyTime();
time=0;
timelimit=15;
LocalLimit=5;
stillboxes=true;
%wipe=false;
if(strcmp(color,'blue'))
 axis=[1,0];   
else
 axis=[-1,0];   
end


coordDynamic=[110,-270,120];
while(stillboxes && time<timelimit)
       time=RossyTime()-t0;
        [dynamicName, dynamicPose, dynamicTwist]=filterOutStaticBlocks();
        if(isempty(dynamicName))
            stillboxes=false;
        else
            stillboxes=true;
        end
        %localtime kicks you out of while loop if you're taking too long
        tl=RossyTime();
        localTime=0;
        close=false;
        while(~close && localTime<LocalLimit && stillboxes &&(time+(LocalLimit/2)<timelimit))
            [dynamicName, dynamicPose, dynamicTwist] = filterOutStaticBlocks();
            if(isempty(dynamicName))
                stillboxes=false;
            else
                stillboxes=true;
            end
%             Love=wesleyLovesMe(dynamicPose);
          
            localTime=RossyTime()-tl; 
            
            r = calculateRadiusForEndEff(lynx,color,axis,0.5);
            if(~isnan(r) && r<50)
                safety=-15;
                [start,~]=findperfect(safety,15);
                JerkMove(lynx,color,start,0.1,1,0.1,2);
                %move(start,lynx);
                close = rotationdriver(lynx,color,r);
%             elseif(Love)
%                 try
%                 close=pickRightSideBlocks(color);
%                 
%                 rosPause(1);
%                 catch
%                     disp("wesley doesn't love me");
%                 end
              else
                
                rosPause(0.3);
                %do reverse rotation driver
            end
            
          
        end
        if(close)
            rosPause(0.5);
            %move block to goal table
           %lynx.set_vel([0,0,0,0,0,-1]);
           q=[0,0,0,0,0,-5];
           lynx.set_pos(q);
           ToleranceMovement(lynx,color,q,0.1,3,0);
           %move(q,lynx);
           q = reachgoal(coordDynamic, color, lynx);
            move(q,lynx);
            %lynx.set_pos(q);
            %ToleranceMovement(lynx,color,q,0.1,3,0);
            move([0,0,0,0,0,0],lynx);
        end


end

end

    
