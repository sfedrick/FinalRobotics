% Fill in this file with your code for Analysis question #4.

% # Discretizations for each joint variable
discStep = 10;

%Link Lengths
L1 = 76.2;
L2 = 146.05;
L3 = 187.325;
L4 = 34;
L5 = 34;
       
%Initialize T matrices, combined them into a single matrix to make easier
%to calculate
T = eye(4);
x = [];
y = [];
z = [];

% Compute the end effector Cartesian coordinate for each permutation of 
% joint variable
joint=6;
lowerLim = [-1.4, -1.2, -1.8, -1.9, -2.0]; % Lower joint limits in radians (grip in mm (negative closes more firmly))
upperLim = [ 1.4,  1.4,  1.7,  1.7,  1.5]; % Upper joint limits in radians (grip in mm)
dims = [lowerLim;upperLim]';
random=100000;
close=0.10;
closerandom=20;
hold on;
plotJointPos([0,0,0,0,0,0], [0.5,0,0],2)\
%walk through config space stochastically
for i=1:random
  qconfig=randpoint(dims);
  J=CreateJac(qconfig,joint);
   J1=round(J(1:3,:));
   J2=round(J(4:6,:),1);
   J=[J1;J2];
    
    if(rank(J)<5)
         closelow=qconfig-close;
         closeup=qconfig+close;
         closedim=[closelow;closeup]';
         %if a singularity is found sample random points near that singular
         %config
        for j=1:closerandom
            qconfig=randpoint(closedim);
            J=CreateJac(qconfig,joint);
            J1=round(J(1:3,:));
            J2=round(J(4:6,:),1);
            J=[J1;J2];
            if(rank(J)<5)
                %plotJointPos(qconfig, [0.5,0,0],2)
                [jointPositions,T0e]=calculateFK(qconfig);
                x=jointPositions(joint,:);
                plot3(x(1),x(2),x(3),'-o','Color','b','MarkerSize',10,'MarkerFaceColor','#D9FFFF');
            end
        end
    end
end
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal;
axis([-260 260 -260 260 -400 500]);
hold off;
