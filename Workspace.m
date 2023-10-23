classdef Workspace
    %WORKSPACE
    %   Class holds all the things needed for running simulations
    
    properties
        dobotMagician;
        dobotCR3;
        rubbish;
        table;
    end

    properties (Access = private)
        rubbishAmount;
        rubbishBounds;
    end
    
    methods
        function self = Workspace()
            self.dobotMagician = Robot("DobotMagician");
            hold on

            self.rubbishAmount = 1;
            self.rubbishBounds = [0 1 0 1 0]; % -x, x, -y, y bounds, and z is constant
        end

        function generateRubbish(self)
            for i = 1:self.rubbishAmount
                % generate random position in the rubbish bounds and place
                % the object there
                randx = self.rubbishBounds(1) + (self.rubbishBounds(2)-self.rubbishBounds(1))*rand();
                randy = self.rubbishBounds(3) + (self.rubbishBounds(4)-self.rubbishBounds(3))*rand();
                z = self.rubbishBounds(5);
                self.rubbish{i} = Rubbish("rand",[randx randy z],i);
            end
            axis equal
        end

        function generateWorkspace(self)
            % just a test bench for now.
            self.table = PlaceObject('bench.ply',[0 0 0]);

            self.dobotMagician.model.base = transl(0,1.2,0.4);
        end
    end
end

