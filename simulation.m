
clear
close all
clc


work = Workspace(3);

magician = Robot("DobotMagician",work.MagicianBaseWorkspace);
magician.animate();

cr3 = Robot("Dobot_CR3",work.CR3BaseWorkspace);
cr3.animate();

work.rubbishModels{1}.moveModel(2,0,0.72);
work.rubbishModels{1}.onConveyer = true;

work.rubbishModels{2}.moveModel(1.5,0,0.72);
work.rubbishModels{2}.onConveyer = true;

work.rubbishModels{3}.moveModel(1,0,0.72);
work.rubbishModels{3}.onConveyer = true;

eStop = false;
steps = 100;

work.animateModels();

conveyerSpeed = 0.006; % change back to 0.003

rubbishAmount = length(work.rubbishModels);

pickupRubbishNumMagician = 0;
index = 0;
wait = false;

input("Press Enter to continue simulation: ")

while rubbishAmount > 0
    if eStop
        magician.eStop = true;
        cr3.eStop = true;
        work.conveyerRunning = false;
        break
    end

    if work.conveyerRunning
        for j = 1:rubbishAmount
            if work.rubbishModels{j}.onConveyer
                rubbishBaseTr = work.rubbishModels{j}.model.base.T;
                
                if rubbishBaseTr(1,4) >= 0.3
                    work.rubbishModels{j}.model.base = rubbishBaseTr * transl(-conveyerSpeed,0,0);
                else
                    work.conveyerRunning = false;
                    pickupRubbishNumMagician = j;
                end
            end
        end
        work.animateModels();

    end
    
    if (pickupRubbishNumMagician > 0) && (magician.pickedUpNum == 0) && (index == 0)
        endTr = work.rubbishModels{pickupRubbishNumMagician}.model.base.T;
        magicianTraj = magician.createTrajIckon(endTr,steps);
        index = 1;
        
    end

    if 0 < index && index <= steps
        magician.changeArmQ(magicianTraj(index,:));
        magician.animate();
        
        % moving picked up bottle with the end effector
        if magician.pickedUpNum > 0
            work.rubbishModels{magician.pickedUpNum}.model.base = magician.robot.model.fkine(magician.armQ).T;
            work.rubbishModels{magician.pickedUpNum}.model.animate(0);
        end

        index = index + 1;

    elseif index > steps
        if magician.pickedUpNum == 0 && pickupRubbishNumMagician > 0 && (wait == false)
            magician.pickedUpNum = pickupRubbishNumMagician;
            magician.rubbishProximity(work.rubbishModels{magician.pickedUpNum});

            if magician.inductiveSensorValue == true % metal can is true, goes to left bin
                endTr = transl(0,0.23,1);
            else
                endTr = transl(0,-0.23,1); % otherwise, goes to right bin
            end

            magicianTraj = magician.createTrajIckon(endTr,steps);
            pickupRubbishNumMagician = 0;
            work.conveyerRunning = true;
            work.rubbishModels{magician.pickedUpNum}.onConveyer = false;
            index = 1;
            wait = true;
        elseif magician.pickedUpNum > 0
            magicianTraj = magician.createTrajIckon(magician.robot.model.fkine(magician.homeQ),steps);
            magicianTraj(end,:) = magician.homeQ;
            index = 1;
            work.rubbishModels{magician.pickedUpNum}.moveModel(0,0,0);
            magician.pickedUpNum = 0;
        else
            rubbishAmount = rubbishAmount - 1;
            magician.pickedUpNum = 0;
            index = 0;
            wait = false;
        end
    end

    pause(0.01)
        
end



