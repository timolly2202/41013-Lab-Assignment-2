
clear
close all
clc
magician = Robot("DobotMagician");

work = Workspace();
work.generateFurniture();
magicianBase = work.MagicianBaseWorkspace;
magician.robot.model.base = magician.robot.model.base.T * transl(magicianBase(1),magicianBase(2), magicianBase(3));
magician.animate
