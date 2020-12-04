function [Tout, change] = WhiteSideUp(Tin, pose)

	%%%%%%

	% Tin is the pose returned by the picked pose function
	% pose is the pose of the block in question; this is obtained from get_object_state
	% z0 is from block pose; y and z are from end effector frame (Tin)
	% If dot(z0,z) = +/- 1, then white side is already facing up
	% otherwise we try to align pick so that dot( z0, y) = + 1
	% change defines the number of steps reqd for aligning white face up
	% change is 0 if white side is already up
	% change is 1; if face can be aligned in 1 step i.e. by picking in a different orientation / no extra steps reqd
	% change is 2; if block needs to be picked and dropped - then picked up again in a different orientation - time consuming step

	%%%%%%

	z0 = pose(1:3,3);
	z0 = z0/norm(z0);
	y = Tin(1:3,2);
	y = y/norm(y);
	z= Tin(1:3, 3);
	z = z/norm(z);
	change = 1;
	
    % Trans matrix for rotating end effector by +/- pi/2 around Z axis
	minusNinety = [ 0, 1, 0, 0;
			-1, 0, 0, 0;
			0, 0, 1, 0;
			0, 0, 0, 1];

	plusNinety = [ 0, -1, 0, 0;
			1, 0, 0, 0;
			0, 0, 1, 0;
			0, 0, 0, 1];

	if abs(dot(z0, z)) == 1                                     
		Tout = Tin;
		change = 0;
		return
	end

	product = dot(z0, y);

	if product > 0.8
		Tout = Tin;
		change = 1;
    elseif (product > -0.2) & (product < 0.2)
		T = Tin * minusNinety;
        productNew = dot(z0, T(1:3, 2));
		if ( productNew > 0.8 )
			Tout = T; change = 1;
			return
		elseif ( productNew < -0.8 )
			Tout = Tin * plusNinety; change = 1;
			return
        end
            
    end
	if product  < -0.8
		change = 2;
		%%%This is where the block would take TWO steps
        %%% This will add a lot of time, we can just ignore this part
        Tout = Tin;
	end

end

 