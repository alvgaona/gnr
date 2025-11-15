clear,clc;
close all;

%start = [0,3,0];
%goal = [25,3,pi/2];
WorldXML = readstruct("jardinRobot3.xml","FileType","xml");
for i=1:size(WorldXML.World.CylindricalPart,2)
    coords = double(split(extractBetween(WorldXML.World.CylindricalPart(i).position,"{","}"),","));
    if strcmp(class(WorldXML.World.CylindricalPart(i).orientation),'string')
        orient = split(extractBetween(WorldXML.World.CylindricalPart(i).orientation,"{","}"),",");
    else
        orient = [0,0,0];
    end
    if WorldXML.World.CylindricalPart(i).nameAttribute == "START"
        start = [coords(1),coords(2),double(orient(3))];
    elseif WorldXML.World.CylindricalPart(i).nameAttribute == "GOAL"
        goal = [coords(1),coords(2),double(orient(3))];
    end
end
ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);
load gardenMap.mat %exampleMaps
map = binaryOccupancyMap(imrotate(garden(:,:,1),180),"Resolution",10);
sv.Map = map;
sv.ValidationDistance = 0.1;
ss.StateBounds = [map.XWorldLimits;map.YWorldLimits; [-pi pi]];
%Create planner
planner = plannerHybridAStar(sv,"MinTurningRadius",1);
refpath = plan(planner,start,goal,"SearchMode","exhaustive"); %refpath.States has the traj coordinates
%% Plot
show(planner);
