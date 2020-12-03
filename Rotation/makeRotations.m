function [wipe] = makeRotations(color)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
global lynx;
lynx = ArmController(color);
pause(1);
time=0;
timelimit=20/0.2;
stillboxes=true;
wipe=false;
%replace tic and toc with ros time
tic
    while(stillboxes && time<timelimit)
       display("outside rotation driver")
       time=toc ;
%         [dynamicName, dynamicPose, dynamicTwist] = filterOutStaticBlocks(lynx);
%         if(isempty(dynamicName))
%             stillboxes=false;
%         else
%             stillboxes=true;
%         end
        
        r=20;
        close=false;
        while(~close)
%            r = calculateRadiusForEndEff(lynx,color);
            close = rotationdriver(lynx,color,r);
            display("inside rotation driver")
            time=toc
        end
        if(close)
            %move block to goal table
          lynx.set_vel([0,0,0,0,0,-100]) 
          lynx.set_pos([0,0,0,0,0,-15])
          ToleranceMovement(lynx,[0,0,0,0,0,-15],0.1,1000);
        end
           
    end
    
   if(stillboxes)
       wipe=true;
   end
    
end

