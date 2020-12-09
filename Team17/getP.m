function [theta1, theta2, theta3, outOfPos] = getP (C, upperLim, lowerLim, constants)
theta1=[];
theta2=[];
theta3=[];
a1 = constants(1);
a2 = constants(2);
a3 = constants(3);
d5=constants(4);
ListJoint=[];


%thetaup
[theta1,theta2,theta3]=elbowDown(C, constants);
ListJoint=[ListJoint;theta1,theta2,theta3];

[theta1,theta2,theta3]=elbowUp(C, constants);
ListJoint=[ListJoint;theta1,theta2,theta3];
[theta1,theta2,theta3]=elbowUpTheta(C, constants,-1);
ListJoint=[ListJoint;theta1,theta2,theta3];
[theta1,theta2,theta3]=elbowUpTheta(C, constants,1);
ListJoint=[ListJoint;theta1,theta2,theta3];
[theta1,theta2,theta3]=elbowDownTheta(C, constants,-1);
ListJoint=[ListJoint;theta1,theta2,theta3];
[theta1,theta2,theta3]=elbowDownTheta(C, constants,1);
ListJoint=[ListJoint;theta1,theta2,theta3];
[row,col]=size(ListJoint);
jointInLimit=[];
for i=1:row
    [output,solution]=limitcheck(ListJoint(i,1:col),upperLim,lowerLim);
    if(solution==1)
        jointInLimit=[jointInLimit;output];
    end
    
end 
[row,col]=size(jointInLimit);
leastnorm=inf;
chooserow=1;
for i=1:row
    q=jointInLimit(i,1:col);
    normq=norm(q);
    if(normq<leastnorm)
        leastnorm=normq;
        chooserow=i;
    end
    
end 
%unpacks valid joints into 
jointInLimitprint= num2cell(jointInLimit(i,1:col));
if(row~=0)
    [theta1,theta2,theta3]=jointInLimitprint{:};
    outOfPos=0;
else
    outOfPos=1;
    theta1=real(theta1);
    theta2=real(theta2);
    theta3=real(theta3);
end
    
%handles singularity
if(C(1)==0 && C(2)==0)
    [theta1,theta2,theta3]=elbowUp(C, constants);
    theta1=nan;
end


end 
