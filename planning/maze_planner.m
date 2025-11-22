function [planner, start, goal, waypoints, traj] = maze_planner(filename, map, inflate)
%MAZE_PLANNER - Path planning for maze navigation using Hybrid A*
%   Computes a collision-free reference path from START to GOAL positions
%   defined in an XML world file using Hybrid A* algorithm on an occupancy
%   map.
%
% Input Arguments:
%   filename (string)    - Path to XML world file containing START and GOAL markers
%   map (H x W x C)      - Binary or grayscale image map (only first channel used)
%   inflate (double)     - Obstacle inflation radius [m] (default: 0.3)
%
% Output Arguments:
%   planner (plannerHybridAStar) - Configured Hybrid A* planner object
%   start (1x3)                  - Start pose [x, y, theta] in world frame [m, m, rad]
%   goal (1x3)                   - Goal pose [x, y, theta] in world frame [m, m, rad]
%   waypoints (Nx3)              - Waypoints along the planned path [x, y, theta]
%   traj (navPath)            - Planned path object containing states and directions
%
% Example:
%   map = imread('maze.png');
%   [planner, start, goal, path] = maze_planner('world.xml', map);
%   show(path);

arguments
    filename {mustBeTextScalar}
    map (:,:,:) {mustBeNumeric}
    inflate (1,1) double {mustBeNonnegative, mustBeFinite} = 0.3
end

%% Parse World XML File
worldxml = readstruct(filename, "FileType", "xml");

start = zeros(1, 3);
goal = zeros(1, 3);
waypoints = [];
it = 1;

% Extract START and GOAL poses from cylindrical parts
for i = 1:size(worldxml.World.CylindricalPart, 2)
    part = worldxml.World.CylindricalPart(i);

    % Extract position coordinates {x, y, z}
    coords = double(split(extractBetween(part.position, "{", "}"), ","));

    % Extract orientation {roll, pitch, yaw}
    if isa(part.orientation, 'string')
        orient = split(extractBetween(part.orientation, "{", "}"), ",");
    else
        orient = [0, 0, 0];
    end

    % Assign to start or goal based on name attribute
    if part.nameAttribute == "START"
        start = [coords(1), coords(2), double(orient(3))];
    elseif part.nameAttribute == "GOAL"
        goal = [coords(1), coords(2), double(orient(3))];
    elseif worldxml.World.CylindricalPart(i).nameAttribute.startsWith("WAYPOINT")
        waypointNum = worldxml.World.CylindricalPart(i).nameAttribute.replace("WAYPOINT","");
        waypoints(it,:) = [coords(1),coords(2),double(orient(3)),double(waypointNum)];
        it=it+1;
    end
end

clear it waypointNum orient coords;
waypoints = sortrows(waypoints,4);

%% Configure State Space and Validator
% SE(2) state space: [x, y, theta]
state_space = stateSpaceSE2;
state_validator = validatorOccupancyMap(state_space);

% Create binary occupancy map from image (rotate 180° for coordinate alignment)
occupancy_map = binaryOccupancyMap(imrotate(map(:,:,1), 180), "Resolution", 10);
occupancy_map.inflate(inflate);

% Configure validator
state_validator.Map = occupancy_map;
state_validator.ValidationDistance = 0.1;

% Set state bounds based on map limits
state_space.StateBounds = [
    occupancy_map.XWorldLimits;
    occupancy_map.YWorldLimits;
    [-pi pi]
];

%% Compute Reference Trajectory

% Hybrid A* planner for car-like robots
planner = plannerHybridAStar(state_validator,"MinTurningRadius",1.6);
if size(waypoints,1) < 1
    refpath = plan(planner,start,goal,"SearchMode","exhaustive"); %refpath.States has the traj coordinates
    traj = refpath.States;
else
    refpath = plan(planner,start,waypoints(1,1:3),"SearchMode","exhaustive");
    traj = refpath.States;
    for i=1:size(waypoints,1)-1
        refpath = plan(planner,waypoints(i,1:3),waypoints(i+1,1:3),"SearchMode","exhaustive");
        traj = [traj; refpath.States];
    end
    refpath = plan(planner,waypoints(size(waypoints,1),1:3),goal,"SearchMode","exhaustive");
    traj = [traj; refpath.States];
end
end