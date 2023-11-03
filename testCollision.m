clear
close all
clc

work = Workspace(4);
% view(3)
% robit.model.animate(robit.homeQ)
% robit.model.teach

[Y,Z] = meshgrid(-0.5:0.05:0.5,0:0.05:0.5);
sizeMat = size(Y);
X = repmat(2.1,sizeMat(1),sizeMat(2));

cubePoints = [X(:),Y(:),Z(:)];

work.cr3.changeArmQ(deg2rad([0 0 90 -70 -180 0]))
work.cr3.animate()

work.cr3.findCollision(cubePoints)


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
        linkCentrePoints(i,:) = [(jointTrs(1,4,i+1) + jointTrs(1,4,i))/2,(jointTrs(2,4,i+1) + jointTrs(2,4,i))/2,(jointTrs(3,4,i+1) + jointTrs(3,4,i))/2];
    end
end

function ellipsoidRadii = getEllipsoidRadii(model) % will only need to calculate once
    ellipsoidRadii = zeros(model.n,3);
    L = model.links;
    for i = 1:model.n % the jointTrs from getJointTr includes the base.
        z = abs(L(i).d)/2 + 0.1;
        x = abs(L(i).a)/2 + 0.1;
        y = 0.1;

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

function jointsInCollision = findCollision(pointCloud,model)
    tr = getJointTr(model);
    elippsoidRadii = getEllipsoidRadii(model);
    jointsInCollision = zeros(1,model.n);
    
    for i = 1:size(tr,3)-1
        pointCloudAndOnes = [inv(tr(:,:,i)) * [pointCloud,ones(size(pointCloud,1),1)]']';
        updatedPointCloud = pointCloudAndOnes(:,1:3);
        % updatedPointCloud = pointCloud.*trotx(roll).*troty(pitch).*trotz(yaw);
        algebraicDistance = GetAlgebraicDist(updatedPointCloud,[0 0 0],elippsoidRadii(i,:));
        if size(find(algebraicDistance < 1),1) > 0
            jointsInCollision(i) = 1;
        end
    end
end