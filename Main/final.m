function final(color)

global lynx;
lynx = ArmController(color);
pause(1);
t0=RossyTime();
time=0;
timelimit=20;
LocalLimit=10;
stillboxes=true;
wipe=false;

axis=[-1,0];
axis=[1,0];
coordDynamic=[110,-275,120];
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
        while(~close && localTime<LocalLimit && stillboxes)
            [dynamicName, dynamicPose, dynamicTwist] = filterOutStaticBlocks();
            if(isempty(dynamicName))
                stillboxes=false;
            else
                stillboxes=true;
            end
            
            
            
            localTime=RossyTime()-tl; 
            
            r = calculateRadiusForEndEff(lynx,color,axis,0.5);
            if(~isnan(r) && r<60)
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
          move(lynx,q);
          q = reachgoal(coordDynamic, color, lynx);
          move(lynx,q);
        end



end