function [close,distance,target] = grip(lynx,color,names,distanceError)
close=false;
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
distance=inf;
target="";
 for i=1:length(listname)
     box=H*pose{listname(i)};
     boxposition=box(1:3,4);
     distancevec=boxposition-EndLocation;
     error=norm(distancevec);
     
     if(error<closestbox)
             closestbox=error;
             target=name{listname(i)};
             distance=distancevec;
     end
 end
  if( closestbox<distanceError)    
      try
      [qnew,isPos]=towardsbox(boxposition(1),boxposition(2),60,q);
      if(isPos)
          %lynx.set_pos(qnew);
          %close=ToleranceMovement(lynx,color,qnew,0.1,1);
%            JerkMove(lynx,color,qnew,0.2,1,0.1,3)
            lynx.command(qnew);
            close=ToleranceMovement(lynx,color,qnew,0.3,2,1);
      end

      catch
      end
  end
                      
           

end

