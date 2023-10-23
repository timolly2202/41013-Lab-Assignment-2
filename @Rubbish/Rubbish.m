classdef Rubbish < handle
    %Rubbish class to hold the bottle and cans the dobot will pick up

    properties
        rubbishType; % type determines what type of rubbish it is, "aluminium", or "plastic", or "glass", or "rand" to randomly choose.
        model;
        workspaceDimensions = [-1 1 -1 1 0 1];
    end

    methods
        function self = Rubbish(rubbishType, basePosition, nameIndex)
            % if no argument added, randomly choose what material the
            % rubbish is
            if nargin < 1
                r = randi([1,2],1); 
                if r == 1
                    rubbishType = 'Can';
                else
                    rubbishType = 'PlasticBottle';
                end
                basePosition = [0 0 0];
                nameIndex = 1;

            elseif nargin < 2
                basePosition = [0 0 0];
                nameIndex = 1;

            elseif nargin < 3
                nameIndex = 1;

            end

            if rubbishType == "rand"
                r = randi([1,2],1); 
                if r == 1
                    rubbishType = 'Can';
                else
                    rubbishType = 'PlasticBottle';
                end
            else
                self.rubbishType = rubbishType;
            end

            % nameIndex is just for naming the robot model with the can and
            % such in it.
            name = ['rubbish', num2str(nameIndex)];
            while ~isempty(findobj('Tag',name)) % checking that the name hasn't already been taken by another object
                nameIndex = nameIndex + 1;
                name = ['rubbish', num2str(nameIndex)];
            end
            
            self.model = self.GetModel(name,rubbishType);
            self.model.base = transl(basePosition(1),basePosition(2),basePosition(3));

            plot3d(self.model,0,'workspace',self.workspaceDimensions,'view',[-30,30],'delay',0,'noarrow','nowrist');

        end

        function delete(self)
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try delete(h.robot); end%#ok<TRYNC>
                try delete(h.wrist); end%#ok<TRYNC>
                try delete(h.link); end%#ok<TRYNC>
                try delete(h); end%#ok<TRYNC>
                try delete(handles); end%#ok<TRYNC>
        end
    end

    methods (Static)
        %% GetModel
        % this function is gotten from the RobotCows class in the UTS
        % modified Peter Corke's Robotics Toolbox
        function model = GetModel(name, rubbishType)

            if strcmp(rubbishType, 'Can')
                plyName = 'can.ply';
            elseif strcmp(rubbishType, 'PlasticBottle')
                plyName = 'bottle.ply';
            end

            [faceData,vertexData] = plyread(plyName,'tri');
            link1 = Link('alpha',0,'a',0,'d',0,'offset',0);
            model = SerialLink(link1,'name',name);
            
            % Changing order of cell array from {faceData, []} to 
            % {[], faceData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.faces = {[], faceData};

            % Changing order of cell array from {vertexData, []} to 
            % {[], vertexData} so that data is attributed to Link 1
            % in plot3d rather than Link 0 (base).
            model.points = {[], vertexData};

            % vertex colours to make it look nice
            % vertexColours = [0.5,0.5,0.5]; % Default if no colours in plyData
            % try 
            %     vertexColours = [plyData.vertex.red ...
            %                     , plyData.vertex.green ...
            %                     , plyData.vertex.blue]/255;
            % 
            % catch ME_1
            %     disp(ME_1);
            %     disp('No vertex colours in plyData');
            % end
            % handles = findobj('Tag', model.name);
            % h = get(handles,'UserData');
            % 
            % h.link(1).Children.FaceVertexCData = vertexColours;
            % h.link(1).Children.FaceColor = 'interp';
        end
        % function rubbishType = randomRubbish()
        %     r = randi([1,2],1); 
        %     if r == 1
        %         rubbishType = 'Can';
        %     else
        %         rubbishType = 'PlasticBottle';
        %     end
        % end
    end
end