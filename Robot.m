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

        pickedUpNum = 0;

        step;
        stepMax = 100;
        
        homeQ;

        lambdaMat; % lambda matrix for RMRC
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
                self.homeQ = deg2rad([0 20 45 90 0]);

            elseif robotName == "Dobot_CR3"
                self.robot = Dobot_CR3();
                self.homeQ = deg2rad([0 -90 0 -90 0 0]);

            else
                self.robot = DobotMagician();
                self.homeQ = deg2rad([0 20 45 90 0]);
            end

            self.robot.model.delay = 0;
            view(3)

            q = self.robot.model.getpos;
            qlims = self.robot.model.qlim;

            lenq = length(q);
            self.armQ = self.homeQ;
            
            self.robot.model.base = transl(robotBaseLocation(1),robotBaseLocation(2),robotBaseLocation(3));
            self.animate;

            self.lambdaMat = 0.01*eye(self.robot.model.n);

        end

        function emergencyStop(self)
            self.eStop = true;
        end

        function emergencyStopReset(self)
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
                    if self.resumeFunction == 1
                        load resume i steps traj; % loads resume state if estop has occured
                    end
                else
                    i = 1;
                    traj = self.createTrajIckon(endTr,steps);
                end

                % deletes it so that it can't be resumed later if it was
                % reinitialized but not resumed.
                try delete resume; end %#ok<TRYNC>
                self.resumeFunction = 0;

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

        function traj = createTrajRMRC(self,endTr,steps,dt)
            x = zeros;
            s = lspb(0,1,steps);
            qMatrix = nan(steps,self.robot.arm.n);
            qMatrix(1,:) = self.robot.armQ;
            
            for i = 1:steps-1
                
            end
        end

        function nextQ = RMRCCalc(dx,dt,q)
            J = self.robot.model.jacob0(q);
            JT = J.';
        
            manipulability = sqrt(det(J*JT));
        
            if manipulability <= 0.05
                JDLS = pinv(JT*J+self.lambdaMat)*JT;
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

        function animateRMRCStep(self,dx,dt)
            self.armQ = self.RMRCCalc(dx,dt);
            self.animate
        end
        
        %% Function to check distance to rubbish object and give sensor readings
        function rubbishProximity(self,rubbish)
            % armPos = self.robot.model.fkine(self.armQ).T;
            % armPos = armPos(1:3,4); % x,y,z position of arm effector

            % rubbishPos = rubbish.model.base.T;
            % rubbishPos = rubbishPos(1:3,4);

            % put the distance check to top of the rubbish, rather than the
            % base
            % rubbishPos(3) = rubbishPos(3) + rubbish.rubbishHeight;
            
            % distance = sqrt(sum((armPos - rubbishPos).^2));

            % disp("Distance from end-effector to rubbish is: " + distance);
            % if distance < 0.05
                if rubbish.rubbishType == "Can"
                    disp("can")
                    self.inductiveSensorValue = true;
                    self.capacitiveSensorValue = false;
                else
                    disp("bottle")
                    self.inductiveSensorValue = false;
                    self.capacitiveSensorValue = true;
                end
            % else
                % self.inductiveSensorValue = false;
                % self.capacitiveSensorValue = false;
            % end
        end
    end
end