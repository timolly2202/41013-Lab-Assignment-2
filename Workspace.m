classdef Workspace
    %WORKSPACE
    %   Class holds all the things needed for running simulations like the
    %   cans,
    
    properties
        rubbishModels;
        table;
    end

    properties (Access = private)
        rubbishAmount;
        rubbishBounds;
    end
    
    methods
        function self = Workspace(rubbishAmount, rubbishBounds)
            % self.dobotMagician = Robot("DobotMagician");
            hold on 

            if nargin < 1
                rubbishAmount = 3;
                rubbishBounds = [0 0.3 0 0.3 0];
            elseif nargin < 2
                rubbishBounds = [0 0.3 0 0.3 0];
            end

            self.rubbishAmount = rubbishAmount;
            self.rubbishBounds = rubbishBounds; % -x, x, -y, y bounds, and z is constant

            for i = 1:self.rubbishAmount
                % generate random position in the rubbish bounds and place
                % the object there
                randx = self.rubbishBounds(1) + (self.rubbishBounds(2)-self.rubbishBounds(1))*rand();
                randy = self.rubbishBounds(3) + (self.rubbishBounds(4)-self.rubbishBounds(3))*rand();
                z = self.rubbishBounds(5);
                self.rubbishModels{i} = Rubbish("rand",[randx randy z],i);
            end
            axis equal
        end

        function generateFurniture(self)
            % just a test bench for now.
            self.table = PlaceObject('bench.ply',[0 0 0]);
        end

        function deleteFurniture(self)
            try delete(self.table); end
        end

    end
end

