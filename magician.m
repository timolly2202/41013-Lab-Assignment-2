classdef magician < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here

    properties
        robot;
        armQ = [0 0.7854 0.7854 0.7854 0];
        defaultArmQ = [0 0.873 0 0 0];
    end

    methods
        function self = magician
            self.robot = DobotMagician();
            self.robot.model.animate(self.armQ)
            view(3)
        end

    end
end