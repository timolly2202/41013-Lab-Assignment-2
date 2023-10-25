classdef Robot < handle
    %Robot 
    %   Detailed explanation goes here

    properties
        robot;
        armQ;

        eStop = false; % function for showing if the estop button has been pressed
        collision = false;
        resumeFunction = 0; % 1 is move arm function
        inductiveSensorValue = false; % value of inductive sensor to identify metallic objects
        capacitiveSensorValue = false; % value of capacitive sensor to identify non-metals

        step;
        stepMax = 100;
    end

    methods
        % constructor requires the specific robot object to be passed
        % through
        function self = Robot(robotName,robotBaseLocation)
            % run(robotName);
            % self.robot = ans;
            if nargin < 2
                robotBaseLocation = [0 0 0];
            end

            if robotName == "DobotMagician"
                self.robot = DobotMagician();

            elseif robotName == "Dobot_CR3"
                self.robot = Dobot_CR3();

            else
                self.robot = DobotMagician();

            end

            self.robot.model.delay = 0;
            view(3)

            q = self.robot.model.getpos;
            qlims = self.robot.model.qlim;

            lenq = length(q);
            self.armQ = zeros(1,lenq);
            
            % This for loop sets the arm joints to the ones generated in
            % the self.robot class, while also checking that it is within
            % the joint limits.
            for i = 1:lenq
                if qlims(i,1)< q(i) && q(i) < qlims(i,2) 
                    self.armQ(i) = q(i);
                else
                    self.armQ(i) = qlims(i,2);
                end
            end
            self.robot.model.base = transl(robotBaseLocation(1),robotBaseLocation(2),robotBaseLocation(3));
            self.animate;

        end
        

        function emergencyStop(self)
            self.eStop = true;
        end

        function resume(self)
            self.resumeFunction = 0;
            self.eStop = false;
        end
        
        % animate function to current joint config
        function animate(self)
            self.robot.model.animate(self.armQ)
        end
        % function to change the armQ
        function changeArmQ(self,q)
            self.armQ = q;
        end

        % Function to move the arm along a specific trajectory
        function moveArm(self, endTr, steps)
                if nargin <= 1
                    load resume i steps traj; % loads resume state if estop has occured
                    try delete resume; end %#ok<TRYNC>
                    self.resume
                else
                    i = 1;
                    traj = self.createTrajIckon(endTr,steps);
                end

                while i <= steps
                    self.changeArmQ(traj(i,:));
    
                    self.animate();

                    pause(0.01);

                    if self.eStop % checking if eStop has been pressed
                         save resume -regexp ^(?!(self)$). % saves all the local variables except the self object
                         self.resumeFunction = 1;
                         return
                    end
                    i = i+1;
                end
        end
        
        %% Trajectory generations
        function traj = createTrajIckon(self,endTr,steps)
            endQ = self.robot.model.ikcon(endTr, self.armQ);
            traj = jtraj(self.armQ,endQ,steps);
        end

        function traj = createTrajRMRC(self, endTr, steps)
            s = lspb(0,1,steps); % Trapezoidal trajectory scalar
            for i=1:steps
                traj(1,i) = (1-s(i))*0.35 + s(i)*0.35; % Points in x
                traj(2,i) = (1-s(i))*-0.55 + s(i)*0.55; % Points in y
                traj(3,i) = 0.5 + 0.2*sin(i*delta); % Points in z
                theta(1,i) = 0; % Roll angle
                theta(2,i) = 5*pi/9; % Pitch angle
                theta(3,i) = 0; % Yaw angle
            end

            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1]; % Create transformation of first point and angle
            q0 = zeros(1,6); % Initial guess for joint angles
            qMatrix(1,:) = p560.ikcon(T,q0); % Solve joint angles to achieve first waypoint

        end
        
        %% Function to check distance to rubbish object and give sensor readings
        function rubbishProximity(self,rubbish)
            armPos = self.robot.model.fkine(self.armQ).T;
            armPos = armPos(1:3,4); % x,y,z position of arm effector

            rubbishPos = rubbish.model.base.T;
            rubbishPos = rubbishPos(1:3,4);

            % put the distance check to top of the rubbish, rather than the
            % base
            if rubbish.rubbishType == "Can"
                rubbishPos(3) = rubbishPos(3) + 0.12;
            else
                rubbishPos(3) = rubbishPos(3) + 0.2;
            end
            
            distance = sqrt(sum((armPos - rubbishPos).^2));

            disp("Distance from end-effector to rubbish is: " + distance);
            if distance < 0.05
                if rubbish.rubbishType == "Can"
                    self.inductiveSensorValue = true;
                    self.capacitiveSensorValue = false;
                else
                    self.inductiveSensorValue = false;
                    self.capacitiveSensorValue = true;
                end
            else
                self.inductiveSensorValue = false;
                self.capacitiveSensorValue = false;
            end
        end
    end
end