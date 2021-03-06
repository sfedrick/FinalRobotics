function [broken] = JerkMove(lynx,color,start,errorJerk,jerkTime,safeError,safeTime)
%a more efficent way to get the end effector to move
    lynx.command(start);
    broken=ToleranceMovement(lynx,color,start,errorJerk,jerkTime,0);
    if(broken)
        lynx.set_vel([0,0,0,0,0,0]);
        lynx.set_pos(start);
        broken=ToleranceMovement(lynx,color,start,safeError,safeTime,0);
    end
end

