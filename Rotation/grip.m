function [close,wait,BoxesInUrFace,speed,distancevec] = grip(lynx,WithInFace,names,color,thresh,speedthresh,v)
close=false;
wait=true;
[q,~]=lynx.get_state();
[RoboLocation,RoboPose] = calculateFK(q);
RoboFinger=RoboPose(1:3,3);
EndLocation=RoboPose(1:3,4);
[name,pose,twist]=lynx.get_object_state();
names=string(names);
listname=[];
%find index for a list of name objects passed into grip
%might be good to break this out into it's own function
for i=1:length(names)
    for j=1:length(name)
        if(strcmp(name(j),names(i)))
            listname(i)=j;
            break;
        end
    end
end

%transform from world to robot frame
if(strcmp(color,'red'))
    H=[1,0,0,200;
       0,1,0,200;
       0,0,1,0;
       0,0,0,1];
elseif(strcmp(color,'blue'))
   H=[-1,0,0,200;
       0,-1,0,200;
       0,0,1,0;
       0,0,0,1]; 
end
closestbox=inf;
 for i=1:length(listname)
     box=H'*pose{listname(i)};
     boxposition=box(1:3,4);
     distancevec=boxposition-EndLocation;
     error=norm(distancevec);
     if(error<closestbox)
         BoxesInUrFace=name(listname(i));
         speed=norm(twist{listname(i)});
         linvel=twist{listname(i)};
         linvel=linvel(1:3);
         linvel(4)=1;
         linvel=H*linvel;
         linvel=linvel(1:3);
         [Side1,Sidebox] = InUrFace(lynx,color,q(6)/2,0.9,1);
         [Side2,Sidebox] = InUrFace(lynx,color,q(6)/2,0.9,-1);
         [vicinity,BoxesInUrV] = InUrVicinity(lynx,color,v);
             closestbox=error;
             %check speed of deisred block and the distance from end
             %effector if both are within some threshold we close 
             %has some problems with obects left and right but not in
             %center of gripper
             if(error<thresh && speed<speedthresh)
                 close=true;
                 wait=false;
                 pause(1);
                 lynx.set_vel([0,0,0,0,0,-100])
                 break
             elseif(abs(distancevec(1))<((q(6))/2)||abs(distancevec(2))<((q(6))/2)||abs(distancevec(3))<15)
                 close=false;
                 wait=true;
                 break
             elseif(((vicinity || WithInFace) && RoboFinger'*linvel<0))
                      try
                      newpos=towardsbox(boxposition(1),boxposition(2),60,q);
                      lynx.set_pos(newpos);
                      ToleranceMovement(lynx,newpos,0.1,1000)
                      catch
                      end
                      wait=true;
                      lynx.set_vel([0,0,0,0,0,0]);
             elseif(RoboFinger'*linvel<0)
                 wait=true;
           
             else
                 wait=false;
             end
           
           
     end
 end
 
 


end
