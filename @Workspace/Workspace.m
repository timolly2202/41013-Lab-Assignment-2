classdef Workspace
    %WORKSPACE
    %   Class holds all the things needed for running simulations like the
    %   cans, and creates the simulated workspace.

    % start of conveyer is [2,0,0.715]
    
    properties
        rubbishModels;
        table;
        MagicianBaseWorkspace = [0,0,0.8];
        CR3BaseWorkspace = [2.5,0,0];
        conveyerRunning = true; % speed of conveyer is around 20 m/min (1.2 kph), and will stop when a can gets close to the magician (0.33 m per second,
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
                rubbishBounds = [2.2 3 -0.5 0.5 0];
            elseif nargin < 2
                rubbishBounds = [2.2 3 -0.5 0.5 0];
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
            self.transformPLY(barrier,trotz(pi/2));
            self.transformPLY(barrier,transl(-1,0,0));

            % belt rotation
            self.transformPLY(belt,trotz(pi/2));
            self.transformPLY(belt,transl(1.1,-1,0));

            % fire extinguisher rotation
            self.transformPLY(fire,trotz(pi/2));
            self.transformPLY(fire,transl(-2.1,-0.6,0));

            % stop button rotation
            self.transformPLY(stopB,trotz(pi));
            self.transformPLY(stopB,transl(-0.1,-0.7,0));
        end

        function animateModels(self)
            for i = 1:length(self.rubbishModels)
                self.rubbishModels{i}.model.animate(0);
            end
        end
    end
    methods(Static)
        function transformPLY(name, transform)
            n = name;
            t = transform;
            
            vertices = get(n,'Vertices'); %getting the vertices from the ply model
            transformedVertices = [vertices,ones(size(vertices,1),1)] * t';
            set(n,'Vertices',transformedVertices(:,1:3));
        end
    end
end

