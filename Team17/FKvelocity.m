% Calculates the linear velocity vector for a given:
% qdot - vector of angular joint velocities
% qconfig - vector of angular joint positions
% joint - joint in the robot that we want to calculate Jacobian for

function [v] = FKvelocity(qdot,qconfig,joint)

    % Create Jacobian for given inputs
    J=CreateJac(qconfig,joint);
    
    % Remove the last entry in qdot
    if(length(qdot)==6)
        qdot(6)=[];
    end
    
    % Transpose the qdot vec if it is wrong transpose
    [row,col]=size(qdot);
    if(col==1)
        v=J*(qdot);
    else
        v=J*(qdot');
    end
end

