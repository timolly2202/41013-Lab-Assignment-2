
clear
close all
clc


work = Workspace(2);

magician = Robot("DobotMagician",work.MagicianBaseWorkspace);
cr3 = Robot("Dobot_CR3",work.CR3BaseWorkspace);

work.rubbishModels{1}.model.base = transl(2,0,0.714);
work.rubbishModels{1}.onConveyer = true;

work.rubbishModels{2}.model.base = transl(1.5,0,0.714);
work.rubbishModels{2}.onConveyer = true;

eStop = false;
steps = 500;

work.animateModels();

conveyerSpeed = 0.003;

rubbishAmount = length(work.rubbishModels);

pickupRubbishNum = 0;

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
                
                if rubbishBaseTr(1,4) >= 0.25
                    work.rubbishModels{j}.model.base = rubbishBaseTr * transl(-conveyerSpeed,0,0);
                else
                    work.conveyerRunning = false;
                    pickupRubbishNum = j;
                end
            end
        end
        work.animateModels();
        pause(0.01)
    end
    
end



