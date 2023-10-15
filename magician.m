classdef magician < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        robot;
        armQ = [0 0.7854 0.7854 0.7854 0];
        defaultArmQ = [0 0.873 0 0 0];

        eStop = false; % function for showing if the estop button has been pressed
        resumeFunction = 0; % 1 is move arm function
    end

    methods
        function self = magician
            self.robot = DobotMagician();
            self.robot.model.delay = 0;
            self.robot.model.animate(self.armQ)
            view(3)
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
                    self.armQ = traj(i,:);
    
                    self.animate

                    pause(0.01);

                    if self.eStop % checking if eStop has been pressed
                         save resume -regexp ^(?!(self)$). % saves all the local variables except the self object
                         self.resumeFunction = 1;
                         return
                    end
                    i = i+1;
                end
        end

        function traj = createTrajIckon(self,endTr,steps)
            endQ = self.robot.model.ikcon(endTr, self.armQ);
            traj = jtraj(self.armQ,endQ,steps);
        end

        %% Test Functions for Emergency Stop file writing and the Estop
        function testWrite(self)
            hello = 1;
            goodbye = 2;
            self.emergencyStop
            save resume -regexp ^(?!(self)$). % saves all the local variables except the self object
        end

        function testRead(self)
            if self.eStop
                load("resume", "goodbye", "hello")
                disp(string(hello))
                disp(string(goodbye))
                self.eStop = false;
                delete("resume.mat")
            else
                disp("not in eStop")
            end
        end

        function testEstop(self)
            tic
            while toc <= 10
                disp(toc)
                pause(1e-4) % need this pause to get the estop to work
                if self.eStop
                    break
                end
            end
        end
        
    end
end