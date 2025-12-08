%% Maze Runner full implementation
clear; close all; clc; rng(0);

load('gardenMap.mat');
map = binaryOccupancyMap(imrotate(garden(:,:,1), 180), "Resolution", 10);

% Map is defined in Hesse form: (d,α).
% d: perpendicular distance to the origin.
% α: angle of that vector whose norm is the perpendicular distance.
hesse_map = HesseMap();

% Add horizontal and vertical lines
hesse_map.addLine(deg2rad(90), 2);
hesse_map.addLine(deg2rad(90), 6);
hesse_map.addLine(deg2rad(0), 0);
hesse_map.addLine(deg2rad(0), 18);

num_walls = hesse_map.num_lines;
map_lines = hesse_map.lines;
%load('garden_lines.mat');
%num_walls = size(map_lines,1);

[~, start, goal, waypoints, traj] = maze_planner('gardenPoli.xml', garden, 0.75);
%start = [0,3,0];
disp('Trajectory calculated!');
%% Vehicle and Simulation Parameters
%Controller params
% controller = NMPCController( ...
%     PredictionHorizon=10, ...
%     ControlHorizon=5, ...
%     TimeStep=0.1, ...
%     StateWeights=[10, 10, 1], ...
%     ControlWeights=[1, 1], ...
%     VelocityLimits=[0, 2], ...
%     AngularLimits=[-2, 2] ...
% );
controller = NMPCCBFController(...
    'HorizonLength', 10, ...
    'TimeStep', 0.1, ...
    'StateWeights', [10, 10, 1], ...
    'ControlWeights', [1, 1], ...
    'VelocityLimits', [0, 2], ...
    'AngularLimits', [-2, 2], ...
    'SafetyRadius', 0.4, ...
    'AlphaCBF', 0.5, ...
    'ScanDownsample', 100, ...
    'ConstraintRange', 3.0, ...
    'MaxIterations', 100);

WorldXML = readstruct("MazeRunner.xml","FileType","xml");
time_step = controller.dt;%0.2       % Discrete time step [s]
pause_time = 0;          % For "realism" use time_step [s]
simulation_time = 40;       % Total simulation time [s]
num_steps = simulation_time / time_step;
robotName = convertStringsToChars(WorldXML.World.Pioneer3ATSim.nameAttribute);%LMS100Sim %LandMark mark_id="1"
laserName = 'LMS100';%convertStringsToChars(WorldXML.World.LMS100Sim.nameAttribute);%'LMS100';
%% Reset Odom
apoloResetOdometry(robotName,start);

%% Define Trajectory
%Run the planner and store the trajectory points in 'traj'!!!
%Also store the initial point in 'start'
pathPoint = 2;%counter for the trajectory points, 1 is start

t = 0:time_step:simulation_time;
% Straight vertical trajectory through center of corridor
v_ref = 0.8;  % Reference speed [m/s]
x_ref = v_ref * t;%zeros(size(t));   % Stay centered in x
y_ref = ones(size(t))*start(2);%v_ref * t % Move upward at constant speed
theta_ref = zeros(size(t));%pi/2 * ones(size(t));    % Point upward (90 degrees)

xref = [x_ref', y_ref', theta_ref'];

%% Beacon Positions
num_beacons = size(WorldXML.World.LandMark,2);
if num_beacons == 0
    beacons = [
        4  8 1;   % Beacon 1
        1  1 2;   % Beacon 2
       11  3 3   % Beacon 3
    ];
    num_beacons=size(beacons, 1);
else %Get beacon positions from file
    beacons = zeros(num_beacons,3);
    for i=1:num_beacons
        coords = split(extractBetween(WorldXML.World.LandMark(i).position,"{","}"),",");
        beacons(i,:) = [coords(1) coords(2) WorldXML.World.LandMark(i).mark_idAttribute];
    end
end
beacons = sortrows(beacons,3); %make index in array coincide with landmark index
num_measurements = 2 * num_beacons;  % Range + bearing per beacon
scan = getLaserScan(laserName);

%% Initial Conditions

% Initial state
if apoloPlaceMRobot(robotName,[start(1),start(2),0],start(3))~=1
    disp("Error placing "+robotName+" on position");
    return
end
apoloLoc = apoloGetLocationMRobot(robotName);%[x y z theta]
true_state = [apoloLoc(1);apoloLoc(2);apoloLoc(4)];%[x y theta]
apoloResetOdometry(robotName,true_state');
apoloUpdate();

% For EKF (in odometry space: Δd, Δβ)
process_noise_d = 9.5003e-05;      % Distance increment noise [m]
process_noise_beta = 3.9080e-05;   % Heading increment noise [rad]
Q = diag([process_noise_d^2, process_noise_beta^2]);

% Pre-compute standard deviations for efficiency
Q_std = sqrt(diag(Q));

% Range and bearing measurements to beacons
measurement_noise_range = 0.018085189925279;   % Range measurement noise [m]
measurement_noise_bearing = 0.023174091647608;  % Bearing measurement noise [rad]

% R matrix: [r1, phi1, r2, phi2, r3, phi3] - 3 beacons, 2 measurements each
R = diag([measurement_noise_range^2, measurement_noise_bearing^2]);

% Pre-compute standard deviations for efficiency
R_std = sqrt(diag(R));

% Initial state prediction (with error for EKF)
estimated_state = true_state + [0.5; 0.5; 0.1];  % Just deviate a little

% Initial state covariance: with uncertainty for the state vector
P = diag([0.5^2, 0.5^2, 0.1^2]);

chi2_threshold = 9.21;  % 99% confidence for 2 DOF
%ekf = EKFLines(estimated_state, P, map_lines, Q, chi2_threshold);
ekf = EKFLandmarks(estimated_state, P, beacons, Q, R);

%% Variables to show results
true_trajectory = zeros(3, num_steps);
estimated_trajectory = zeros(3, num_steps);
variance_history = zeros(3, num_steps);
control_history = zeros(2, num_steps);

%% Main Simulation Loop

for step = 1:num_steps
    % 1. Simulate ground truth robot motion: Differential drive model with
    % linear and angular velocity as control input, [v, ω]

    % Differential drive control inputs: linear velocity (v) and angular velocity (ω)
    %If we are close to the point in the trajectory, switch to the next one
    Ax=traj(pathPoint,1)-ekf.x(1);
    Ay=traj(pathPoint,2)-ekf.x(2);
    if norm([Ax,Ay]) <= 0.5 %if distance to traj. point is less than ...
        if pathPoint == size(traj,1)
            break %we ended on the last point
        end
        pathPoint = pathPoint + 1; %switch to next point
    end
    
    % Compute control using:
    % x: current state
    % xref: from current iteration onwards
    % scan: measurements scanned in current iteration
    u = controller.compute(ekf.x, traj(pathPoint,:),scan);%xref(step, :)
    v = u(1); omega = u(2);
    %[v, omega] = differential_drive_pid(gainStruct,traj(pathPoint,:),estimated_state,time_step);

    % Differential drive dynamics (no noise - perfect execution)
    % Note: v and ω could come from wheel velocities: v = (v_R + v_L)/2, ω = (v_R - v_L)/b

    % Use Euler method for integration (simpler, first-order)
    theta_k = true_state(3);

    % Update robot with specified commands
    prev_odom= apoloGetOdometry(robotName);
    apoloMoveMRobot(robotName,[v omega],time_step);
    apoloUpdate();
    pause(pause_time);
    apoloLoc = apoloGetLocationMRobot(robotName);%[x y z theta]
    true_state = [apoloLoc(1);apoloLoc(2);apoloLoc(4)];%[x y theta]
    true_trajectory(:, step) = true_state;

    %% Generate Measurements (Range and Bearing to each beacon) - Vectorized
    % Compute all ranges and bearings at once
    lm_meas = apoloGetLaserLandMarks(laserName);
    measurementIDs = lm_meas.id;
    num_measurements=size(measurementIDs,2)*2;
    measurements = zeros(num_measurements, 1);
    measurements(1:2:end) = lm_meas.distance';
    measurements(2:2:end) = lm_meas.angle';
    % Normalize bearing angles to [-pi, pi]
    measurements(2:2:end) = atan2(sin(measurements(2:2:end)), cos(measurements(2:2:end)));

    scan = getLaserScan(laserName);

    %% EKF PREDICTION STEP
    % Control input: u = [Δd, Δβ]
    [delta_d_hat,delta_beta_hat]=apolo_odometry(robotName,prev_odom);
    
    ekf.predict(delta_d_hat, delta_beta_hat);

    %% EKF UPDATE STEP - Vectorized
    % Lines are observed in the Hessse form
    %lines_observed = ransac_lines(scan, 0.015, 3);  
    
    %fprintf('[DEBUG]: %d lines_observed \r',size(lines_observed,1));

    %ekf.update(lines_observed);

    if ~isempty(measurements)
        ekf.update([measurements(1:2:end),measurements(2:2:end)],measurementIDs);
    end

    %% Store Results
    estimated_trajectory(:, step) = ekf.x;
    variance_history(:, step) = sqrt(diag(ekf.P));
end

% Adapt storage in case we ended early
if step < num_steps
    true_trajectory = true_trajectory(:,1:step-1);
    estimated_trajectory = estimated_trajectory(:,1:step-1);
end

%% Compute Estimation Errors
position_error = sqrt(sum((true_trajectory(1:2,:) - estimated_trajectory(1:2,:)).^2, 1));
angle_error = abs(true_trajectory(3,:) - estimated_trajectory(3,:));

%% Visualization
figure('Name', 'EKF with Range+Bearing Measurements', 'Position', [50 50 1400 900]);

% Plot 1: 2D Trajectory with Beacons
subplot(2,3,1);
hold on; grid on; axis equal;
plot(true_trajectory(1,:), true_trajectory(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'True');
plot(estimated_trajectory(1,:), estimated_trajectory(2,:), 'r--', 'LineWidth', 2, 'DisplayName', 'Estimated');
plot(true_trajectory(1,1), true_trajectory(2,1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
xlabel('X [m]'); ylabel('Y [m]');
title('Trajectory');
legend('Ground truth', 'Estimated', 'Initial Position', 'Location', 'best');

% Plot 2: X Position
subplot(2,3,2);
plot(true_trajectory(1,:), 'b-', 'LineWidth', 1.5); hold on;
plot(estimated_trajectory(1,:), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('X [m]');
title('X Position'); legend('Ground truth', 'Estimated');
grid on;

% Plot 3: Y Position
subplot(2,3,3);
plot(true_trajectory(2,:), 'b-', 'LineWidth', 1.5); hold on;
plot(estimated_trajectory(2,:), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Y [m]');
title('Y Position'); legend('Ground truth', 'Estimated');
grid on;

% Plot 4: Heading Angle
subplot(2,3,4);
plot(rad2deg(true_trajectory(3,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(rad2deg(estimated_trajectory(3,:)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Heading [deg]');
title('Heading Angle'); legend('Ground truth', 'Estimated');
grid on;

% Plot 5: Position Error
subplot(2,3,5);
plot(position_error, 'm-', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Position Error [m]');
title('Position Estimation Error');
grid on;

% Plot 6: Covariance
subplot(2,3,6);
semilogy(sqrt(variance_history(1,:)), 'r-', 'LineWidth', 1.5); hold on;
semilogy(sqrt(variance_history(2,:)), 'g-', 'LineWidth', 1.5);
semilogy(sqrt(variance_history(3,:)), 'b-', 'LineWidth', 1.5);
xlabel('Time Step'); ylabel('Standard Deviation');
title('State Uncertainty');
legend('\sigma_x', '\sigma_y', '\sigma_\theta');
grid on;

% Plot 7: 2D Trajectories with Beacons
figure("Name","Trajectories");
hold on; grid on; axis equal;
show(map);
plot(traj(:,1), traj(:,2), ':pentagramy', 'LineWidth', 2, 'DisplayName', 'Planned');
plot(traj(pathPoint,1), traj(pathPoint,2), ':pentagramr', 'LineWidth', 2, 'DisplayName', 'Planned');
plot(true_trajectory(1,:), true_trajectory(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'True');
plot(estimated_trajectory(1,:), estimated_trajectory(2,:), 'r--', 'LineWidth', 2, 'DisplayName', 'Estimated');
plot(true_trajectory(1,1), true_trajectory(2,1), 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
plot(waypoints(:,1), waypoints(:,2), 'ko', 'MarkerSize', 12, 'MarkerFaceColor', 'cyan');
xlabel('X [m]'); ylabel('Y [m]');
title('Trajectory');
legend('Planned', 'pathPoint', 'Ground truth', 'Estimated', 'Initial Position', 'Waypoints', 'Location', 'best');

%% Display Statistics
fprintf('\n========== EKF with range+bearing measurements - Results ==========\n');
fprintf('True Dynamics:          Differential Drive [v, ω]\n');
fprintf('EKF Control Input:      Odometry [Δd, Δβ]\n');
fprintf('Time Step:              %.2f s\n', time_step);
fprintf('Simulation Time:        %.2f s\n', simulation_time);
fprintf('Number of Steps:        %d\n', num_steps);
fprintf('\nMeasurement Noise:\n');
fprintf('\nFinal Position Error:   %.3f m\n', position_error(end));
fprintf('Mean Position Error:    %.3f m\n', mean(position_error));
fprintf('Max Position Error:     %.3f m\n', max(position_error));
fprintf('Final Angle Error:      %.3f deg\n', rad2deg(angle_error(end)));
fprintf('\nFinal Std Dev (x):      %.4f m\n', sqrt(variance_history(1,end)));
fprintf('Final Std Dev (y):      %.4f m\n', sqrt(variance_history(2,end)));
fprintf('Final Std Dev (theta):  %.4f rad\n', sqrt(variance_history(3,end)));
fprintf('=================================================================\n\n');

function [scan] = getLaserScan(laserName)
    %% Get LiDAR scan
    scan = apoloGetLaserData(laserName);%lms_scan_new(x, obstacles, max_range, noise_std, lidar_model);
    b=size(scan);                 %LMS200->181 measures, last one is always 0, 180º
    ang = 0:b(2)-1;
    if b(2) > 181
        ang = (ang-270)*(1.5*pi/b(2));
    else
	    scan = scan(1:180);
        ang = 0:179;
        ang = (ang-90)*(pi/b(2));
    end
    scan = [scan(:), ang(:)];
end
