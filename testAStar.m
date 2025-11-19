clear,clc;
close all;

%start = [0,3,0];
%goal = [25,3,pi/2];
waypoints = [];
it = 1;
WorldXML = readstruct("gardenPoli.xml","FileType","xml");
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
    elseif WorldXML.World.CylindricalPart(i).nameAttribute.startsWith("WAYPOINT")
        waypointNum = WorldXML.World.CylindricalPart(i).nameAttribute.replace("WAYPOINT","");
        waypoints(it,:) = [coords(1),coords(2),double(orient(3)),double(waypointNum)];
        it=it+1;
    end
end
clear it waypointNum orient coords
waypoints = sortrows(waypoints,4);
ss = stateSpaceSE2;
sv = validatorOccupancyMap(ss);
load gardenMap.mat %exampleMaps
map = binaryOccupancyMap(imrotate(garden(:,:,1),180),"Resolution",10);
map.inflate(0.3);
sv.Map = map;
sv.ValidationDistance = 0.1;
ss.StateBounds = [map.XWorldLimits;map.YWorldLimits; [-pi pi]];
%Create planner
planner = plannerHybridAStar(sv,"MinTurningRadius",1.6);
if size(waypoints,1) < 1
    refpath = plan(planner,start,goal,"SearchMode","exhaustive"); %refpath.States has the traj coordinates
    traj = refpath.States;
    %% Plot
    show(planner);
else
    refpath = plan(planner,start,waypoints(1,1:3),"SearchMode","exhaustive");
    traj = refpath.States;
    %% Plot
    figure;
    show(planner);title("start to way1");
    for i=1:size(waypoints,1)-1
        refpath = plan(planner,waypoints(i,1:3),waypoints(i+1,1:3),"SearchMode","exhaustive");
        traj = [traj; refpath.States];
        figure;show(planner);title("way"+ i +" to way"+ i+1);
    end
    refpath = plan(planner,waypoints(size(waypoints,1),1:3),goal,"SearchMode","exhaustive");
    traj = [traj; refpath.States];
    figure;show(planner);title("way"+ size(waypoints,1) +" to goal");
end
