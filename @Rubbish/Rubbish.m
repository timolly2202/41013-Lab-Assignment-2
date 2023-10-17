classdef Rubbish
    %Rubbish class
    %   plastic bottle model by Mike Simulation from https://www.turbosquid.com/3d-models/3d-plastic-water-bottle-model-1369732
    %   can model by DanJ08 from https://www.turbosquid.com/3d-models/metal-can-3d-model-1960980

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