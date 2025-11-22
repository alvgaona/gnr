# Guiado y Navegación de Robots
This repository contains all the code developed for the group assignment in the 'Guiado y Navegación de Robots' subject. It was bounded to the [Apolo simulator](https://github.com/mhernando/Apolo).

# Requirements
- MATLAB (tested on R2023b)
- [Apolo](https://github.com/mhernando/Apolo) for any scripts under the [apolo](/apolo) and [calibration](/calibration) folder, as they make calls to the simulator.

# Run the mobile robot sim
1. Open Apolo and the map you want to test the algorithm on ([this one](/apolo/world/MazeRunner.mwf) is recommended)
2. Open MATLAB and add the MATLAB folder under your Apolo installation to the path
3. Add [apolo](/apolo), [control](/control) and [planning](/planning) to the MATLAB path also
4. Run [`apolo_full.m`](/apolo/apolo_full.m). The default parameters should work fine, but most configurable ones are at the top of the script
> [!TIP]
> The trajectory the robot will try to follow can be defined in the map file, but to be able to read it in MATLAB, the world needs to be in XML format. Apolo can also load worlds in XML, but for our maps, collisions with meshparts stop working. To define a trajectory in a map:
> 1. Create two cylinders, named START and STOP
> 2. (Optional) Create as many other cylinders, that will define midpoints, as you want, and call them WAYPOINT1, WAYPOINT2, etc.
> 2. Change their position and yaw to define points in the trajectory
> 3. Change their height so the robot wont collide with them during simulation
> 4. Save the world in Apolo, by clicking File->Save World XML
> 5. Pass this file to the `maze_planner` in [`apolo_full.m`](/apolo/apolo_full.m)

# Authors

- **Alvaro J. Gaona** - [@alvgaona](https://github.com/alvgaona)
- **Lucas Gomez-Velayos** - [@Pigamer37](https://github.com/Pigamer37)
- **Peter Pasuy-Quevedo** - [@peter2395](https://github.com/peter2395)