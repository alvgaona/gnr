# Maze Runner: A Differential Mobile Robot for Maze Navigation

## Prerequisites

- MATLAB (>=R2023b)
- [CasAdi for MATLAB](https://github.com/casadi/casadi/releases/tag/3.7.2)
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

### Standalone MATLAB

Standalone scripts are simulations based entirely on MATLAB.
These scripts are essentially built to tests algorithms and integrations without relying on Apolo.
The nonlinear MPC is mostly implemented in CasAdi so the folder downloaded from [casadi/casadi](https://github.com/casadi/casadi) release needs to be added to the MATLAB path.

- [`ekf_lines.m`](standalone/ekf_lines.m): runs an EKF with RANSAC line extraction.
- [`ekf_range_bearing.m`](standalone/ekf_range_bearing.m): runs an EKF with landmarks (range and bearing model).
- [`nmpc_trajectory.m`](standalone/nmpc_trajectory.m): runs a nonlinear MPC trajectory tracker.
- [`nmpc_cbf_corridor.m`](standalone/nmpc_cbf_corridor.m): runs a nonlinear MPC trajectory tracker with Control Barrier Functions (CBF) for obstacle avoidance.
- [`nmpc_cbf_ekf_lines_corridor.m`](standalone/nmpc_cbf_ekf_lines_corridor.m): runs a nonlinear MPC trajectory tracker with obstacle avoidance and EKF lines.
- [`nmpc_cbf_ekf_lines.m`](standalone/nmpc_cbf_ekf_lines.m): runs a nonlinear MPC with an EKF lines on a full and long trajectory.
- [`nmpc_cbf_ekf_range_brearing.m`](standalone/nmpc_cbf_ekf_range_bearing.m): runs a nonlinear MPC with an EKF range and bearing on a full and long trajectory.
- [`garden_extract_lines.m`](standalone/garden_extract_lines.m): extract lines from an occupancy grid map in Hesse form.
- [`pid_trajectory.m`](standalone/pid_trajectory.m): runs a PID trajectory tracker.
