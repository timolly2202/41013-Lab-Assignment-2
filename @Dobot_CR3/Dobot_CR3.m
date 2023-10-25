classdef Dobot_CR3 < RobotBaseClass

    properties(Access = public)
        plyFileNameStem = 'CR3';
    end

    methods
%% Define robot Function 
        function self = Dobot_CR3(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(0,0,0);  
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
          
            self.CreateModel();
            self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
            self.PlotAndColourRobot();
    
            drawnow
                end

%% Create the robot model
        function CreateModel(self)  

            % Create the CR3 model mounted on a linear rail
            link(1) = Link('d',0.1348,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(2) = Link('d',0,'a',-0.274,'alpha',0,'qlim',deg2rad([-180 0]), 'offset',0);
            link(3) = Link('d',0,'a',-0.23,'alpha',0,'qlim', deg2rad([-155 155]), 'offset',0);
            link(4) = Link('d',-0.1288,'a',0,'alpha',pi/2,'qlim', deg2rad([-360 360]), 'offset', 0);
            link(5) = Link('d',0.116,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            link(6) = Link('d',0.105,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset',0);
          
			% Generate the model
            self.model = SerialLink(link,'name',self.name);       
        end
    end
end