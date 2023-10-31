clear
close all
clc

robit = Dobot_CR3;
view(3)
robit.model.animate(robit.homeQ)
% robit.model.teach


jointTrs = getJointTr(robit.model)
elippsoidRadii = getEllipsoidRadii(robit.model)

centrePoints = getLinkCentrePoints(jointTrs)
hold on
for i = 1:robit.model.n
    [X,Y,Z] = ellipsoid( 0, 0, 0, ...
        elippsoidRadii(i,1),elippsoidRadii(i,2), elippsoidRadii(i,3));
    robit.model.points{i+1} = [X(:),Y(:),Z(:)];
    warning off
    robit.model.faces{i+1} = delaunay(robit.model.points{i+1});
    warning on;
end
robit.model.plot3d(robit.homeQ)
%% Functions
% Get the transform of every joint (i.e. start and end of every link),
% function created from Q2.4 in lab 5 solution
function jointTrs = getJointTr(model) 
    jointTrs = zeros(4,4,model.n+1);
    jointTrs(:,:,1) = model.base;
    L = model.links;
    q = model.getpos();
    for i = 1 : model.n
        jointTrs(:,:,i+1) = jointTrs(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
    end
end

function linkCentrePoints = getLinkCentrePoints(jointTrs)
linkCentrePoints = zeros(length(jointTrs)-1,3);
    for i = 1:length(jointTrs) -1 % the jointTrs from getJointTr includes the base.
        linkCentrePoints(i,:) = [(jointTrs(1,4,i+1) + jointTrs(1,4,i))/2,(jointTrs(2,4,i+1) + jointTrs(2,4,i))/2,(jointTrs(1,4,i+1) + jointTrs(3,4,i))/2];
    end
end

function ellipsoidRadii = getEllipsoidRadii(model) % will only need to calculate once
    ellipsoidRadii = zeros(model.n,3);
    L = model.links;
    for i = 1:model.n % the jointTrs from getJointTr includes the base.
        z = abs(L(i).d)/2 + 0.15;
        x = abs(L(i).a)/2 + 0.15;
        y = 0.15;

        ellipsoidRadii(i,:) = [x,y,z];
    end
end

% function from 41013 lab 6 solution to check the algebraic distance of
% points in a mesh to the ellipsoid around a link. (<1 is a collision with
% the vertex).
function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)
algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
+ ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
+ ((points(:,3)-centerPoint(3))/radii(3)).^2;
end