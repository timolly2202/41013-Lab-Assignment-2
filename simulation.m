
clear
close all
clc


work = Workspace(2);
work.generateFurniture();

magician = Robot("DobotMagician",work.MagicianBaseWorkspace);

work.rubbishModels{1}.model.base = transl(2,0,0.714);
work.rubbishModels{1}.onConveyer = true;

work.rubbishModels{2}.model.base = transl(1.5,0,0.714);
work.rubbishModels{2}.onConveyer = true;

eStop = false;
steps = 500;

work.animateModels();

conveyerSpeed = 0.003;

for t = 1:steps
    if work.conveyerRunning == true
        for j = 1:length(work.rubbishModels)
            if work.rubbishModels{j}.onConveyer
                rubbishBaseTr = work.rubbishModels{j}.model.base.T;
                
                if rubbishBaseTr(1,4) >= 0.25
                    work.rubbishModels{j}.model.base = rubbishBaseTr * transl(-conveyerSpeed,0,0);
                else
                    work.conveyerRunning = false;
                end
            end
        end
        work.animateModels();
        pause(0.01)
    end
end

if eStop
    magician.eStop = true
    work.conveyerRunning = false
end