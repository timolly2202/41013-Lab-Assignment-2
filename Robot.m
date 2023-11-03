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

        gamepad = false;
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
                self.homeQ = deg2rad([0 -90 0 -90 -180 0]);

            else
                self.robot = DobotMagician();
                self.homeQ = deg2rad([0 20 45 90 0]);
            end

            self.robot.model.delay = 0;

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
        function traj = createTrajIckon(self,endTr,steps,startQ)
            if nargin < 2
                startQ = self.armQ;
            end
            endQ = self.robot.model.ikcon(endTr, startQ);
            traj = jtraj(startQ,endQ,steps);
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
                if rubbish.rubbishType == "can"
                    % disp("can")
                    self.inductiveSensorValue = true;
                    self.capacitiveSensorValue = false;
                else
                    % disp("bottle")
                    self.inductiveSensorValue = false;
                    self.capacitiveSensorValue = true;
                end
            % else
                % self.inductiveSensorValue = false;
                % self.capacitiveSensorValue = false;
            % end
        end

        %% Functions for collision detection
        function jointTrs = getJointTr(self) 
            jointTrs = zeros(4,4,self.robot.model.n+1);
            jointTrs(:,:,1) = self.robot.model.base;
            L = self.robot.model.links;
            q = self.robot.model.getpos();
            for i = 1 : self.robot.model.n
                jointTrs(:,:,i+1) = jointTrs(:,:,i) * trotz(q(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            end
        end
        
        function linkCentrePoints = getLinkCentrePoints(self)
            jointTrs = self.getJointTr;
            linkCentrePoints = zeros(length(jointTrs)-1,3);
            for i = 1:length(jointTrs) -1 % the jointTrs from getJointTr includes the base.
                linkCentrePoints(i,:) = [(jointTrs(1,4,i+1) + jointTrs(1,4,i))/2,(jointTrs(2,4,i+1) + jointTrs(2,4,i))/2,(jointTrs(3,4,i+1) + jointTrs(3,4,i))/2];
            end
        end
        
        function ellipsoidRadii = getEllipsoidRadii(self) % will only need to calculate once
            ellipsoidRadii = zeros(self.robot.model.n,3);
            L = self.robot.model.links;
            for i = 1:self.robot.model.n % the jointTrs from getJointTr includes the base.
                z = abs(L(i).d)/2 + 0.1;
                x = abs(L(i).a)/2 + 0.1;
                y = 0.1;
        
                ellipsoidRadii(i,:) = [x,y,z];
            end
        end
        
        function jointsInCollision = findCollision(self,pointCloud)
            tr = self.getJointTr();
            elippsoidRadii = self.getEllipsoidRadii();
            jointsInCollision = zeros(1,self.robot.model.n);
            
            for i = 1:size(tr,3)-1
                pointCloudAndOnes = [inv(tr(:,:,i)) * [pointCloud,ones(size(pointCloud,1),1)]']';
                updatedPointCloud = pointCloudAndOnes(:,1:3);
                % updatedPointCloud = pointCloud.*trotx(roll).*troty(pitch).*trotz(yaw);
                algebraicDistance = self.GetAlgebraicDist(updatedPointCloud,[0 0 0],elippsoidRadii(i,:));
                if size(find(algebraicDistance < 1),1) > 0
                    jointsInCollision(i) = 1;
                end
            end
        end

        function gamePadControl(self)
            id = 1;
            joy = vrjoystick(id);
            dt = 0.07;
            while self.gamepad
                [axes,buttons,povs] = read(joy);
                q = self.armQ;
                % 1 - turn joystick input into an end-effector velocity command
                Kv = -0.2;
                Kw = -0.5;
                
                stickDrift = 0.15;

                if abs(axes(2))> stickDrift
                    vx = Kv*axes(2); % left joystick up down
                else
                    vx = 0;
                end
                
                if abs(axes(1))> stickDrift
                    vy = Kv*axes(1); % left joystick left right
                else
                    vy = 0;
                end
                
                if abs(axes(3))> stickDrift
                    vz = Kv*axes(3); % triggers
                else
                    vz = 0;
                end
                
                if abs(axes(5))> stickDrift
                    wx = Kw*axes(5); % right joystick up down
                else
                    wx = 0;
                end
            
                if abs(axes(4))> stickDrift
                    wy = Kw*axes(4); % right joystick left right
                else
                    wy = 0;
                end
            
                wz = 0; % Kw*axes(); %
            
                dx = [vx;vy;vz;wx;wy;wz];
            
                % 2 - use J inverse to calculate joint velocity
                J = self.robot.model.jacob0(q);
                JT = J.';
            
                manipulability = sqrt(det(J*JT));
                % manipulability = 0
                if manipulability <= 0.05
            
                    % DLS Jacobian
                    JDLS = pinv(JT*J+self.lambdaMat)*JT;
                    dq = JDLS*dx;
                else
                    dq = pinv(J)*dx;
                end
            
                % 3 - apply joint velocity to step robot joint angles 
                q = q + (dq.')*dt;
                % -------------------------------------------------------------
                % Update plot
                self.changeArmQ(q);
                self.animate()

                pause(0.001)
            end
        end

    end
    methods(Static)
        % function from 41013 lab 6 solution to check the algebraic distance of
        % points in a mesh to the ellipsoid around a link. (<1 is a collision with
        % the vertex).
        function algebraicDist = GetAlgebraicDist(points, centerPoint, radii)
        algebraicDist = ((points(:,1)-centerPoint(1))/radii(1)).^2 ...
        + ((points(:,2)-centerPoint(2))/radii(2)).^2 ...
        + ((points(:,3)-centerPoint(3))/radii(3)).^2;
        end
    end
end