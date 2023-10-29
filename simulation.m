clear
close all
clc

work = Workspace(3);

% steps for trajectories
magicianSteps = 50; 
cr3Steps = 100;

conveyerSpeed = 0.006;

totalRubbish = length(work.rubbishModels);
rubbishAmount = totalRubbish;

magicianPickupRubbishNum = 0;
magicianIndex = 0;
magicianWait = false;

cr3Index = 0;
cr3NextRubbish = 1;
cr3PickupRubbishNum = 0;
cr3Wait = false;

try delete(text_h); end %#ok<TRYNC>
message = sprintf(['inductive sensor: ', num2str(false),'\n','capcaitive sensor: ', num2str(false)]);
text_h = text(0,0,1.3, message,'FontSize', 10);


%%
input("Press Enter to continue simulation: ")

% checks to see how much rubbish is put in the bin and stops loop
while rubbishAmount > 0

    % checks if estop is pressed at every step, and stops the
    % simulation if it has been.
    if work.eStop
        work.magician.eStop = true;
        work.cr3.eStop = true;
        work.conveyerRunning = false;
        save resume -regexp ^(?!(self)$). % saves all the local variables except the self object
        break
    end
    
    % This if statement stops the conveyer if a piece of rubbish at the end
    % point (simulated light sensor essentially), as well as simulating the
    % movement of all the rubbish items on the conveyer
    if work.conveyerRunning
        for j = 1:totalRubbish
            if work.rubbishModels{j}.onConveyer 
                rubbishBaseTr = work.rubbishModels{j}.model.base.T;
                
                if rubbishBaseTr(1,4) >= 0.27
                    work.rubbishModels{j}.model.base = rubbishBaseTr * transl(-conveyerSpeed,0,0);
                else
                    work.conveyerRunning = false;
                    magicianPickupRubbishNum = j;
                end
            end
        end
        work.animateRubbishModels();

    end
    
    % generating the initial trajectory for the magician to pick up the
    % rubbish at the end of the conveyer
    if (magicianPickupRubbishNum > 0) && (work.magician.pickedUpNum == 0) && (magicianIndex == 0)
        endTr = work.rubbishModels{magicianPickupRubbishNum}.model.base.T;
        magicianTraj = work.magician.createTrajIckon(endTr,magicianSteps);
        magicianIndex = 1;
        
    end
    
    % moving the magician when there is a trajectory to follow
    if 0 < magicianIndex && magicianIndex <= magicianSteps
        work.magician.changeArmQ(magicianTraj(magicianIndex,:));
        work.magician.animate();
        
        % moving picked up bottle with the end effector if it is picked up
        if work.magician.pickedUpNum > 0
            work.rubbishModels{work.magician.pickedUpNum}.model.base = work.magician.robot.model.fkine(work.magician.armQ).T;
            work.rubbishModels{work.magician.pickedUpNum}.model.animate(0);
        end

        magicianIndex = magicianIndex + 1;
    
    
    elseif magicianIndex > magicianSteps
        % when the magician has moved to the rubbish and now has to pick it
        % up.
        if work.magician.pickedUpNum == 0 && magicianPickupRubbishNum > 0 && ~magicianWait
            work.magician.pickedUpNum = magicianPickupRubbishNum;
            
            % checking the simulated inductive sensor to see if the object
            % is a metal can or plastic water bottle.
            work.magician.rubbishProximity(work.rubbishModels{work.magician.pickedUpNum});
            
            try delete(text_h); end %#ok<TRYNC>
            message = sprintf(['inductive sensor: ', num2str(work.magician.inductiveSensorValue),'\n','capcaitive sensor: ', num2str(work.magician.capacitiveSensorValue)]);
            text_h = text(0,0,1.3, message,'FontSize', 10);

            if work.magician.inductiveSensorValue == true % metal can is true, goes to left bin
                endTr = transl(-0.1,0.24,0.7);
            else
                endTr = transl(-0.1,-0.24,0.7); % otherwise, goes to right bin
            end
            magicianTraj = work.magician.createTrajIckon(endTr,magicianSteps);
            magicianPickupRubbishNum = 0;

            work.conveyerRunning = true;
            work.rubbishModels{work.magician.pickedUpNum}.onConveyer = false;
            magicianIndex = 1;
            magicianWait = true;

        % letting go of the picked up rubbish as it is above the bin and
        % moving the arm back to the home position
        elseif work.magician.pickedUpNum > 0
            magicianTraj = work.magician.createTrajIckon(work.magician.robot.model.fkine(work.magician.homeQ),magicianSteps);
            magicianTraj(end,:) = work.magician.homeQ;
            magicianIndex = 1;

            work.rubbishModels{work.magician.pickedUpNum}.model.base = transl(0,0,0.2);
            work.rubbishModels{work.magician.pickedUpNum}.model.animate(0);
            
            try delete(text_h); end %#ok<TRYNC>
            message = sprintf(['inductive sensor: ', num2str(false),'\n','capcaitive sensor: ', num2str(false)]);
            text_h = text(0,0,1.3, message,'FontSize', 10);

            work.magician.pickedUpNum = 0;
        
        % resetting the arm as it is back home, ready to pick up the next
        % piece of rubbish.
        else
            work.animateRubbishModels;
            rubbishAmount = rubbishAmount - 1;
            work.magician.pickedUpNum = 0;
            magicianIndex = 0;
            magicianWait = false;
        end
    end
    
    % creating trajectory of cr3 to pick up next piece of rubbish
    if (work.cr3.pickedUpNum == 0) && (cr3Index == 0)
        if cr3NextRubbish <= totalRubbish
            endTr = work.rubbishModels{cr3NextRubbish}.model.base.T;
            cr3Traj = work.cr3.createTrajIckon(endTr,cr3Steps);
            cr3Index = 1;
        end
    end
    
    % moving cr3 along trajectory
    if 0 < cr3Index && cr3Index <= cr3Steps
        work.cr3.changeArmQ(cr3Traj(cr3Index,:));
        work.cr3.animate();
        
        % moving picked up bottle with the end effector if it is picked up
        if work.cr3.pickedUpNum > 0
            work.rubbishModels{work.cr3.pickedUpNum}.model.base = work.cr3.robot.model.fkine(work.cr3.armQ).T;
            work.rubbishModels{work.cr3.pickedUpNum}.model.animate(0);
        end

        cr3Index = cr3Index + 1;
    
    elseif cr3Index > cr3Steps
        % Let go of rubbish at end of trajectory
        if (work.cr3.pickedUpNum > 0)
            work.rubbishModels{work.cr3.pickedUpNum}.model.base = transl(2, 0, 0.5+work.rubbishModels{work.cr3.pickedUpNum}.rubbishHeight);
            work.rubbishModels{work.cr3.pickedUpNum}.onConveyer = true;
            work.cr3.pickedUpNum = 0;
            cr3Index = 0;
        
        % pickup rubbish and place on conveyer
        else
            work.cr3.pickedUpNum = cr3NextRubbish;
            cr3NextRubbish = cr3NextRubbish +1;

            endTr = transl(2, 0, 0.5+work.rubbishModels{work.cr3.pickedUpNum}.rubbishHeight);

            cr3Traj = work.cr3.createTrajIckon(endTr,cr3Steps);
            cr3Index = 1;
        end
    end

    pause(0.01);    
end



