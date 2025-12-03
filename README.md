# Maze Runner: A Differential Mobile Robot for Maze Navigation

## Prerequisites

- MATLAB (>=R2023b)
- [Apolo](https://github.com/mhernando/Apolo) for any scripts under the [apolo](/apolo)
folder, as these are integrated with the simulator.

## Usage

Maze Runnner can be simulated with a Apolo (3D simulator) or standalone MATLAB with fewer capabilities.

### Apolo and MATLAB

The very first thing to do is to pull up Apolo.
Opening the desired world is necessary for the MATLAB scripts to integrate with it.
The recommended Apolo world is [`MazeRunner.mwf`](apolo/world/MazeRunner.mwf).
A maze (garden) and the mobile robot should be visible and placed correctly.

At this point, MATLAB is the second part of this.
All scripts and algorithms that interact directly with Apolo are in the `apolo/` directory of the project.
It leverages other modules, e.g., control, dynamics, planning; these modules can be used without Apolo.
The entry point of this simulation should be [`apolo_full.m`](apolo/apolo_full.m), which connects
everything and even shows visual plots related to the simulation.
It has several parameters but most of the default ones should work out of the box.

> [!TIP]
> The trajectory the robot will try to follow can be defined in the map file, but to be able to read it in MATLAB, the world needs to be in XML format. Apolo can also load worlds in XML, but for our maps, collisions with meshparts stop working. To define a trajectory in a map:
> 1. Create two cylinders, named START and STOP
> 2. (Optional) Create as many other cylinders, that will define midpoints, as you want, and call them WAYPOINT1, WAYPOINT2, etc.
> 2. Change their position and yaw to define points in the trajectory
> 3. Change their height so the robot wont collide with them during simulation
> 4. Save the world in Apolo, by clicking File->Save World XML
> 5. Pass this file to the `maze_planner` in [`apolo_full.m`](/apolo/apolo_full.m)

### Standalone MATLAB

WIP
