classdef Workspace
    %WORKSPACE
    %   Class holds all the things needed for running simulations like the
    %   cans, and creates the simulated workspace.

    % start of conveyer is [2,0,0.5]
    
    properties
        rubbishModels;
        conveyerRunning = true; % speed of conveyer is around 20 m/min (1.2 kph), and will stop when a can gets close to the magician (0.33 m per second,
        
        eStop = false;
        resumeFunction = 0; % 1 is for simulation

        magician;
        cr3;
    end

    properties (Access = private)
        rubbishAmount;
        rubbishRadius; % radius that the rubbish can be placed in around robot (length of arm basically)

        magicianBaseLocation = [0,0,0.5];
        cr3BaseLocation = [2.3,0,0.15];
    end
    
    methods
        function self = Workspace(rubbishAmount, rubbishRadius)
            % self.dobotMagician = Robot("DobotMagician");
            hold on
            self.generateFurniture();

            if nargin < 1
                rubbishAmount = 3;
                rubbishRadius = 0.5;
            elseif nargin < 2
                rubbishRadius = 0.5;
            end

            self.rubbishAmount = rubbishAmount;
            self.rubbishRadius = rubbishRadius;

            z = 0;
            centrePoints = [self.cr3BaseLocation(1)+0.1,self.cr3BaseLocation(2)];

            for i = 1:self.rubbishAmount
                % generate random position in the rubbish bounds and place
                % the object there
                randTheta = rand()*pi;
                randLength = rand()*rubbishRadius;
                x = centrePoints(1)+randLength*sin(randTheta);
                y = centrePoints(2)+randLength*cos(randTheta); % y is sin as it can be positive or negative around the centre point
                self.rubbishModels{i} = Rubbish("rand",[x y z],i);
            end

            self.magician = Robot("DobotMagician",self.magicianBaseLocation);
            self.magician.animate();
            
            self.cr3 = Robot("Dobot_CR3",self.cr3BaseLocation);
            self.cr3.animate();

            axis equal
        end

        function generateFurniture(self)
            %% Filling the Scene

            hold on

            %Concrete from canvas

            % Floor
            surf([-1.5,-1.5;3,3],[-1.5,1;-1.5,1],[0,0;0,0],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            
            % Wall
            surf([-1.5,-1.5;-1.5,-1.5],[-1.5,1;-1.5,1],[0,0;2,2],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

            % support slab for magician
            surf([-0.4,-0.4;0.1,0.1],[-0.65,0.65;-0.65,0.65],[0.1,0.1;0.1,0.1],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([-0.4,-0.4;-0.4,-0.4],[-0.65,0.65;-0.65,0.65],[0.1,0.1;0,0],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([0.1,0.1;0.1,0.1],[-0.65,0.65;-0.65,0.65],[0.1,0.1;0,0],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([-0.4,-0.4;0.1,0.1],[-0.65,-0.65;-0.65,-0.65],[0.1,0;0.1,0],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([-0.4,-0.4;0.1,0.1],[0.65,0.65;0.65,0.65],[0.1,0;0.1,0],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

            % additional robot support slab magician
            surf([-0.1,-0.1;0.1,0.1],[-0.1,0.1;-0.1,0.1],[0.2,0.2;0.2,0.2],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([-0.1,-0.1;-0.1,-0.1],[-0.1,0.1;-0.1,0.1],[0.2,0.2;0.1,0.1],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([0.1,0.1;0.1,0.1],[-0.1,0.1;-0.1,0.1],[0.2,0.2;0.1,0.1],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([-0.1,-0.1;0.1,0.1],[-0.1,-0.1;-0.1,-0.1],[0.1,0.2;0.1,0.2],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([-0.1,-0.1;0.1,0.1],[0.1,0.1;0.1,0.1],[0.1,0.2;0.1,0.2],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            
            % support slab cr3
            surf([2.2,2.2;2.4,2.4],[-0.1,0.1;-0.1,0.1],[0.15,0.15;0.15,0.15],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([2.2,2.2;2.2,2.2],[-0.1,0.1;-0.1,0.1],[0.15,0.15;0,0],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([2.4,2.4;2.4,2.4],[-0.1,0.1;-0.1,0.1],[0.15,0.15;0,0],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([2.2,2.2;2.4,2.4],[-0.1,-0.1;-0.1,-0.1],[0,0.15;0,0.15],'CData',imread('concrete.jpg'),'FaceColor','texturemap');
            surf([2.2,2.2;2.4,2.4],[0.1,0.1;0.1,0.1],[0,0.15;0,0.15],'CData',imread('concrete.jpg'),'FaceColor','texturemap');

            %Placing objects from UTS
            PlaceObject('tableRound0.3x0.3x0.3m.ply',[0.12,-0.03,0.5-0.3]);
            fire = PlaceObject('fireExtinguisher.ply',[-0.6,-1.1,0]);
            barrier = PlaceObject('barrier1.5x0.2x1m.ply',[0,0,0]);
            stopB = PlaceObject('emergencyStopWallMounted.ply',[0,0,0.25]);
            % person = PlaceObject('personMaleConstruction.ply',[0,0,-0.5]);

            %Placing objects from external sources
            belt = PlaceObject('conveyor.ply',[1,0,0]); %found at https://sketchfab.com/3d-models/simple-rubber-conveyor-0819b51c59c3407cb98f0e2c75029e30
            bin1 = PlaceObject('plasticBin.ply',[-0.15,0.4,0.6-0.3]); %found at https://sketchfab.com/3d-models/plastic-bin-fa922bc1d2b143bda5bc82f881b982d8
            bin2 = PlaceObject('plasticBin.ply',[-0.15,-0.4,0.6-0.3]);

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

        function animateRubbishModels(self)
            for i = 1:length(self.rubbishModels)
                try self.rubbishModels{i}.model.animate(0); end %#ok<TRYNC>
            end
        end

        function emergencyStop(self)
            self.eStop = true;
            self.conveyerRunning = false;
            self.magician.emergencyStop();
            self.cr3.emergencyStop();
        end

        function emergencyStopReset(self)
            self.eStop = false;
            self.conveyerRunning = true;
            self.magician.emergencyStopReset();
            self.cr3.emergencyStopReset();
        end
        
        function simulation(self)
            
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

