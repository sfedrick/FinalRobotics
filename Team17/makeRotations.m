function [wipe] = makeRotations(color)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
global lynx;
lynx = ArmController(color);
pause(1);
t0=RossyTime();
time=0;
timelimit=20;
LocalLimit=10;
stillboxes=true;
wipe=false;
    while(stillboxes && time<timelimit)
       time=RossyTime()-t0; 
      
        [dynamicName, dynamicPose, dynamicTwist] = filterOutStaticBlocks(lynx);
        if(isempty(dynamicName))
            stillboxes=false;
        else
            stillboxes=true;
        end
        %localtime kicks you out of while loop if you're taking too long
        tl=RossyTime();
        localTime=0;
        close=false;
        while(~close && localTime<LocalLimit && stillboxes)
            [dynamicName, dynamicPose, dynamicTwist] = filterOutStaticBlocks(lynx);
            if(isempty(dynamicName))
                stillboxes=false;
            else
                stillboxes=true;
            end
            localTime=RossyTime()-tl; 
            safety=-15;
            [start,~]=findperfect(safety,15);
            JerkMove(lynx,color,start,0.3,1,0.1,5);
            r = calculateRadiusForEndEff(lynx,color,[ 1,0],0.5);
            if(~isnan(r))
                close = rotationdriver(lynx,color,r);
            else
                rosPause(1);
                %do reverse rotation driver
            end
            
          
        end
        if(close)
            %move block to goal table
          lynx.set_vel([0,0,0,0,0,-100]);
          q=[0,0,0,0,0,-5];
          lynx.set_pos(q);
          ToleranceMovement(lynx,color,q,0.1,5,0)
          
        end
           
    end
    
   if(stillboxes)
       wipe=true;
   end
    
end

