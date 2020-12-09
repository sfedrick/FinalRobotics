% This function calculates the Jacobian matrix for the following inputs:
% q - 6x1 joint position vector
% pointinterest - the joint of interest that we want to calculate Jacobian
%                 for (b/w 1-6)

function [J] = CreateJac(q,pointinterest)

% Calculates FK and gets the z-axis of the homogeneous transf matrix
% for the current configuration
[jointPositions,zaxis]=currentConfig(q);

% Initialize Jacobian
J=[];

% Only continue if 
if(pointinterest>0)
    
    % Gets the real space position of the point of interest
    Oe=jointPositions(pointinterest,:);
    
    % Create the Jacobian
    for i=1:pointinterest-1
      zi_1= zaxis(:,i);
      
      % Create the skew symmetric matrix
      Si=createS(zi_1);
      Oi=Oe-jointPositions(i,:);
      
      % linear velocity
      Jvi=Si*Oi';
      
      % angular velocity
      Jwi=zi_1;
      
      % combine into Ji and append to main J
      Ji=[Jvi;Jwi];
      J=[J,Ji];
  
    end
    
   % Special case if pointinterest =1 
   if(pointinterest==1)
       Jvi=[0,0,0]';
       Jwi=[0,0,0]';
       J=[Jvi;Jwi];
   end
end
end

