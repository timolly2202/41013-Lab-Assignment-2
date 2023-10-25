classdef Workspace
    %WORKSPACE
    %   Class holds all the things needed for running simulations like the
    %   cans,
    
    properties
        rubbishModels;
        table;
    end

    properties (Access = private)
        rubbishAmount;
        rubbishBounds;
    end
    
    methods
        function self = Workspace(rubbishAmount, rubbishBounds)
            % self.dobotMagician = Robot("DobotMagician");
            hold on 

            if nargin < 1
                rubbishAmount = 3;
                rubbishBounds = [0 0.3 0 0.3 0];
            elseif nargin < 2
                rubbishBounds = [0 0.3 0 0.3 0];
            end

            self.rubbishAmount = rubbishAmount;
            self.rubbishBounds = rubbishBounds; % -x, x, -y, y bounds, and z is constant

            for i = 1:self.rubbishAmount
                % generate random position in the rubbish bounds and place
                % the object there
                randx = self.rubbishBounds(1) + (self.rubbishBounds(2)-self.rubbishBounds(1))*rand();
                randy = self.rubbishBounds(3) + (self.rubbishBounds(4)-self.rubbishBounds(3))*rand();
                z = self.rubbishBounds(5);
                self.rubbishModels{i} = Rubbish("rand",[randx randy z],i);
            end
            axis equal
        end

        function generateFurniture(self)
            %% Filling the Scene

            hold on

            %Concrete from canvas

            % Floor
            surf([-1.5,-1.5;3,3],[-1.5,1;-1.5,1],[0,0;0,0],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

            % support slab
            surf([-0.4,-0.4;0.1,0.1],[-0.65,0.65;-0.65,0.65],[0.4,0.4;0.4,0.4],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([-0.4,-0.4;-0.4,-0.4],[-0.65,0.65;-0.65,0.65],[0.4,0.4;0,0],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([0.1,0.1;0.1,0.1],[-0.65,0.65;-0.65,0.65],[0.4,0.4;0,0],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([-0.4,-0.4;0.1,0.1],[-0.65,-0.65;-0.65,-0.65],[0.4,0;0.4,0],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([-0.4,-0.4;0.1,0.1],[0.65,0.65;0.65,0.65],[0.4,0;0.4,0],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

            % additional robot support slab
            surf([-0.1,-0.1;0.1,0.1],[-0.1,0.1;-0.1,0.1],[0.5,0.5;0.5,0.5],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([-0.1,-0.1;-0.1,-0.1],[-0.1,0.1;-0.1,0.1],[0.5,0.5;0.4,0.4],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([0.1,0.1;0.1,0.1],[-0.1,0.1;-0.1,0.1],[0.5,0.5;0.4,0.4],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([-0.1,-0.1;0.1,0.1],[-0.1,-0.1;-0.1,-0.1],[0.4,0.5;0.4,0.5],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([-0.1,-0.1;0.1,0.1],[0.1,0.1;0.1,0.1],[0.4,0.5;0.4,0.5],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

            %Placing objects from UTS
            PlaceObject('tableRound0.3x0.3x0.3m.ply',[0.12,-0.03,0.5]);
            fire = PlaceObject('fireExtinguisher.ply',[-0.6,-1.1,0]);
            barrier = PlaceObject('barrier1.5x0.2x1m.ply',[0,0,0]);
            stopB = PlaceObject('emergencyStopWallMounted.ply',[0,0,0.25]);
            % person = PlaceObject('personMaleConstruction.ply',[0,0,-0.5]);

            %Placing objects from external sources
            belt = PlaceObject('conveyor.ply',[1,0,0]); %found at https://sketchfab.com/3d-models/simple-rubber-conveyor-0819b51c59c3407cb98f0e2c75029e30
            bin1 = PlaceObject('plasticBin.ply',[-0.15,0.4,0.6]); %found at https://sketchfab.com/3d-models/plastic-bin-fa922bc1d2b143bda5bc82f881b982d8
            bin2 = PlaceObject('plasticBin.ply',[-0.15,-0.4,0.6]);

            % axis equal;

            %Barrier rotations
            transformPLY(barrier,trotz(pi/2));
            transformPLY(barrier,transl(-1,0,0));

            % belt rotation
            transformPLY(belt,trotz(pi/2));
            transformPLY(belt,transl(1.1,-1,0));

            % fire extinguisher rotation
            transformPLY(fire,trotz(pi/2));
            transformPLY(fire,transl(-2.1,-0.6,0));

            % stop button rotation
            transformPLY(stopB,trotz(pi));
            transformPLY(stopB,transl(-0.1,-0.7,0));
        end

        function deleteFurniture(self)
            try delete(self.table); end
        end

    end
end

