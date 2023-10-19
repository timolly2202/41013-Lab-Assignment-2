classdef Rubbish
    %Rubbish class to hold the bottle and cans the dobot will pick up

    properties
        material; % type determines what type of rubbish it is, "aluminium", or "plastic", or "glass"
        baseTr;
    end

    methods
        function self = Rubbish()
            self.material = "aluminium";

        end

        function outputArg = method1(obj,inputArg)
            outputArg = obj.material + inputArg;
        end
    end
end