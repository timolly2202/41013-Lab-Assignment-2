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
                rubbishRadius = 0.4;
            elseif nargin < 2
                rubbishRadius = 0.4;
            end

            self.rubbishAmount = rubbishAmount;
            self.rubbishRadius = rubbishRadius;

            z = 0;
            centrePoints = [self.cr3BaseLocation(1)+0.1,self.cr3BaseLocation(2)];

            for i = 1:self.rubbishAmount
                % generate random position in the rubbish bounds and place
                % the object there
                randTheta = rand()*pi;
                randLength = 0.1+rand()*self.rubbishRadius;
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
            % steps for trajectories
            magicianSteps = 50; 
            cr3Steps = 100;
            
            conveyerSpeed = 0.006;
            
            totalRubbish = length(self.rubbishModels);
            rubbishLeft = totalRubbish;
            
            magicianPickupRubbishNum = 0;
            magicianIndex = 0;
            magicianWait = false;
            
            cr3Index = 0;
            cr3NextRubbish = 1;

            
            try delete(text_h); end %#ok<TRYNC>
            message = sprintf(['inductive sensor: ', num2str(false),'\n','capcaitive sensor: ', num2str(false)]);
            text_h = text(0,0,1.3, message,'FontSize', 10);
            
            % checks to see how much rubbish is put in the bin and stops loop
            while rubbishLeft > 0
            
                % checks if estop is pressed at every step, and stops the
                % simulation if it has been.
                if self.eStop
                    self.magician.eStop = true;
                    self.cr3.eStop = true;
                    self.conveyerRunning = false;
                    save resume -regexp ^(?!(self)$). % saves all the local variables except the self object
                    break
                end
                
                % This if statement stops the conveyer if a piece of rubbish at the end
                % point (simulated light sensor essentially), as well as simulating the
                % movement of all the rubbish items on the conveyer
                if self.conveyerRunning
                    for j = 1:totalRubbish
                        if self.rubbishModels{j}.onConveyer 
                            rubbishBaseTr = self.rubbishModels{j}.model.base.T;
                            
                            if rubbishBaseTr(1,4) >= 0.27
                                self.rubbishModels{j}.model.base = rubbishBaseTr * transl(-conveyerSpeed,0,0);
                            else
                                self.conveyerRunning = false;
                                magicianPickupRubbishNum = j;
                            end
                        end
                    end
                    self.animateRubbishModels();
            
                end
                
                % generating the initial trajectory for the magician to pick up the
                % rubbish at the end of the conveyer
                if (magicianPickupRubbishNum > 0) && (self.magician.pickedUpNum == 0) && (magicianIndex == 0)
                    endTr = self.rubbishModels{magicianPickupRubbishNum}.model.base.T;
                    magicianTraj = self.magician.createTrajIckon(endTr,magicianSteps);
                    magicianIndex = 1;
                    
                end
                
                % moving the magician when there is a trajectory to follow
                if 0 < magicianIndex && magicianIndex <= magicianSteps
                    self.magician.changeArmQ(magicianTraj(magicianIndex,:));
                    self.magician.animate();
                    
                    % moving picked up bottle with the end effector if it is picked up
                    if self.magician.pickedUpNum > 0
                        self.rubbishModels{self.magician.pickedUpNum}.model.base = self.magician.robot.model.fkine(self.magician.armQ).T;
                        self.rubbishModels{self.magician.pickedUpNum}.model.animate(0);
                    end
            
                    magicianIndex = magicianIndex + 1;
                
                
                elseif magicianIndex > magicianSteps
                    % when the magician has moved to the rubbish and now has to pick it
                    % up.
                    if self.magician.pickedUpNum == 0 && magicianPickupRubbishNum > 0 && ~magicianWait
                        self.magician.pickedUpNum = magicianPickupRubbishNum;
                        
                        % checking the simulated inductive sensor to see if the object
                        % is a metal can or plastic water bottle.
                        self.magician.rubbishProximity(self.rubbishModels{self.magician.pickedUpNum});
                        
                        try delete(text_h); end %#ok<TRYNC>
                        message = sprintf(['inductive sensor: ', num2str(self.magician.inductiveSensorValue),'\n','capcaitive sensor: ', num2str(self.magician.capacitiveSensorValue)]);
                        text_h = text(0,0,1.3, message,'FontSize', 10);
            
                        if self.magician.inductiveSensorValue == true % metal can is true, goes to left bin
                            endTr = transl(-0.1,0.24,0.7);
                        else
                            endTr = transl(-0.1,-0.24,0.7); % otherwise, goes to right bin
                        end
                        magicianTraj = self.magician.createTrajIckon(endTr,magicianSteps);
                        magicianPickupRubbishNum = 0;
            
                        self.conveyerRunning = true;
                        self.rubbishModels{self.magician.pickedUpNum}.onConveyer = false;
                        magicianIndex = 1;
                        magicianWait = true;
            
                    % letting go of the picked up rubbish as it is above the bin and
                    % moving the arm back to the home position
                    elseif self.magician.pickedUpNum > 0
                        magicianTraj = self.magician.createTrajIckon(self.magician.robot.model.fkine(self.magician.homeQ),magicianSteps);
                        magicianTraj(end,:) = self.magician.homeQ;
                        magicianIndex = 1;
            
                        self.rubbishModels{self.magician.pickedUpNum}.model.base = transl(0,0,0.2);
                        self.rubbishModels{self.magician.pickedUpNum}.model.animate(0);
                        
                        try delete(text_h); end %#ok<TRYNC>
                        message = sprintf(['inductive sensor: ', num2str(false),'\n','capcaitive sensor: ', num2str(false)]);
                        text_h = text(0,0,1.3, message,'FontSize', 10);
            
                        self.magician.pickedUpNum = 0;
                    
                    % resetting the arm as it is back home, ready to pick up the next
                    % piece of rubbish.
                    else
                        self.animateRubbishModels;
                        rubbishLeft = rubbishLeft - 1;
                        self.magician.pickedUpNum = 0;
                        magicianIndex = 0;
                        magicianWait = false;
                    end
                end
                
                % creating trajectory of cr3 to pick up next piece of rubbish
                if (self.cr3.pickedUpNum == 0) && (cr3Index == 0)
                    if cr3NextRubbish <= totalRubbish
                        endTr = self.rubbishModels{cr3NextRubbish}.model.base.T;
                        cr3Traj = self.cr3.createTrajIckon(endTr,cr3Steps);
                        cr3Index = 1;
                    end
                end
                
                % moving cr3 along trajectory
                if 0 < cr3Index && cr3Index <= cr3Steps
                    self.cr3.changeArmQ(cr3Traj(cr3Index,:));
                    self.cr3.animate();
                    
                    % moving picked up bottle with the end effector if it is picked up
                    if self.cr3.pickedUpNum > 0
                        self.rubbishModels{self.cr3.pickedUpNum}.model.base = self.cr3.robot.model.fkine(self.cr3.armQ).T;
                        self.rubbishModels{self.cr3.pickedUpNum}.model.animate(0);
                    end
            
                    cr3Index = cr3Index + 1;
                
                elseif cr3Index > cr3Steps
                    % Let go of rubbish at end of trajectory
                    if (self.cr3.pickedUpNum > 0)
                        self.rubbishModels{self.cr3.pickedUpNum}.model.base = transl(2, 0, 0.5+self.rubbishModels{self.cr3.pickedUpNum}.rubbishHeight);
                        self.rubbishModels{self.cr3.pickedUpNum}.onConveyer = true;
                        self.cr3.pickedUpNum = 0;
                        cr3Index = 0;
                    
                    % pickup rubbish and place on conveyer
                    else
                        self.cr3.pickedUpNum = cr3NextRubbish;
                        cr3NextRubbish = cr3NextRubbish +1;
            
                        endTr = transl(2, 0, 0.5+self.rubbishModels{self.cr3.pickedUpNum}.rubbishHeight);
            
                        cr3Traj = self.cr3.createTrajIckon(endTr,cr3Steps);
                        cr3Index = 1;
                    end
                end
            
                pause(0.01);    
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

