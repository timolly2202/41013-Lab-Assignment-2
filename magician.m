classdef magician < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        robot;
        armQ = [0 0.7854 0.7854 0.7854 0];
        defaultArmQ = [0 0.873 0 0 0];

        eStop = false; % function for showing if the estop button has been pressed
        resumeSection = 0;
    end

    methods
        function self = magician
            self.robot = DobotMagician();
            self.robot.model.animate(self.armQ)
            view(3)
        end
        
        function emergencyStop(self)
            self.eStop = true;
        end

        function animate(self)
            self.robot.model.animate(self.armQ)
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