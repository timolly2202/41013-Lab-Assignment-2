clear
close all
clc

work = Workspace(5);

steps = 100;
dt = 0.01;

x = zeros(6,steps);
s = lspb(0,1,steps);
qMatrix = zeros(steps,work.cr3.robot.model.n);
qMatrix(1,:) = work.cr3.armQ;
 
% x1 = extractX(work.cr3.robot.model.fkine(work.cr3.armQ).T)
% x2 = extractX(work.rubbishModels{1}.model.base.T)

x1 = extractX(work.cr3.robot.model.fkine(work.cr3.armQ).T)
x2 = [2.5,-0.5,0.2,0,0,0]

for i = 1:steps-1
    x(:,i) = x1*(1-s(i)) + s(i)*x2;
end

for i = 1:steps-1
    qMatrix(i+1,:) = RMRCCalc(x(:,i+1),x(:,i),dt,qMatrix(i,:),work);
end

for i = 1:steps-1
    work.cr3.changeArmQ(qMatrix(i,:));
    work.cr3.animate();
    pause(dt)
end

%% functions
function nextQ = RMRCCalc(x2,x1,dt,q,work)
    
    dx = (x2 - x1)/dt;
    J = work.cr3.robot.model.jacob0(q);
    JT = J.';

    manipulability = sqrt(det(J*JT));

    if manipulability <= 0.05
        JDLS = pinv(JT*J+work.cr3.lambdaMat)*JT;
        dq = JDLS*dx;
    else
        dq = pinv(J)*dx;
    end
    nextQ = q+(dq.')*dt;
end

function x = extractX(tr)
    x = tr(1:3,4);
    x = x.';
    angles = [0,0,0];
    % angles = extractAngles(tr);
    x = [x,angles];
end

function rollPitchYaw = extractAngles(tr)
    roll = atan2(tr(3,2),tr(3,3)); % phi
    yaw = atan2(tr(2,1),tr(1,1)); % psi
    
    if cos(yaw) == 0
        pitch = atan2(-tr(3,1),(tr(2,1)/sin(yaw)));
    else
        pitch = atan2(-tr(3,1),(tr(1,1)/cos(yaw)));
    end
    rollPitchYaw = [roll,pitch,yaw];
end