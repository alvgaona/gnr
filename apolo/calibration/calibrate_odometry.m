%% Odometry Calibration Script
% Calibrates odometry sensors by measuring statistical properties of
% odometry increments over repeated motion commands.
%
% This script:
%   1. Resets odometry to zero
%   2. Sends N identical velocity commands
%   3. Measures odometry increments [Δd, Δβ] for each step
%   4. Computes mean and standard deviation for noise characterization
%
% Output:
%   - Displays calibration statistics
%   - Plots histograms of measured increments
%
% Usage:
%   Modify parameters below, then run this script in MATLAB.

clear; clc;

%% Simulation Parameters

% Robot and simulation settings
robot_name = 'Marvin';
dt = 0.02;                  % Time step [s]
num_measurements = 100;     % Number of samples to collect

% Velocity command to test (set one to 0 to test individually)
linear_vel_cmd = 0.0;       % Linear velocity command [m/s]
angular_vel_cmd = 0.1;      % Angular velocity command [rad/s]

%% Initialize

% Storage for measurements [delta_d; delta_beta]
measurements = zeros(2, num_measurements);

% Reset odometry to known state
apoloResetOdometry(robot_name);

%% Collect Calibration Data

fprintf('Collecting %d odometry measurements...\n', num_measurements);

for i = 1:num_measurements
    % Get current odometry reading
    prev_odometry = apoloGetOdometry(robot_name);

    % Send velocity command
    apoloMoveMRobot(robot_name, [linear_vel_cmd, angular_vel_cmd], dt);
    apoloUpdate();

    % Compute odometry increment
    [delta_d, delta_beta] = calculateOdometryDiff(robot_name, prev_odometry);

    % Store measurements
    measurements(1, i) = delta_d;
    measurements(2, i) = delta_beta;
end

fprintf('Data collection complete.\n\n');

%% Compute Statistics

% Distance increment statistics
mean_delta_d = mean(measurements(1, :));
std_delta_d = std(measurements(1, :));

% Heading increment statistics
mean_delta_beta = mean(measurements(2, :));
std_delta_beta = std(measurements(2, :));

%% Display Results

fprintf('========== Odometry Calibration Results ==========\n');
fprintf('Robot:                 %s\n', robot_name);
fprintf('Time step:             %.3f s\n', dt);
fprintf('Linear velocity cmd:   %.3f m/s\n', linear_vel_cmd);
fprintf('Angular velocity cmd:  %.3f rad/s (%.2f deg/s)\n', ...
    angular_vel_cmd, rad2deg(angular_vel_cmd));
fprintf('Number of samples:     %d\n\n', num_measurements);

fprintf('Distance Increment (Δd):\n');
fprintf('  Mean:                %.6f m\n', mean_delta_d);
fprintf('  Std Dev:             %.6f m\n', std_delta_d);
fprintf('  Expected (v*dt):     %.6f m\n\n', linear_vel_cmd * dt);

fprintf('Heading Increment (Δβ):\n');
fprintf('  Mean:                %.6f rad (%.4f deg)\n', ...
    mean_delta_beta, rad2deg(mean_delta_beta));
fprintf('  Std Dev:             %.6f rad (%.4f deg)\n', ...
    std_delta_beta, rad2deg(std_delta_beta));
fprintf('  Expected (ω*dt):     %.6f rad (%.4f deg)\n', ...
    angular_vel_cmd * dt, rad2deg(angular_vel_cmd * dt));
fprintf('===================================================\n\n');

%% Visualization

figure('Name', 'Odometry Calibration Results', 'Position', [100 100 1200 500]);

% Distance increment histogram
subplot(1, 2, 1);
histogram(measurements(1, :), 20, 'FaceColor', [0.2 0.6 0.8]);
hold on;
xline(mean_delta_d, '--r', 'LineWidth', 2);
xlabel('Distance Increment Δd [m]');
ylabel('Frequency');
title('Distance Increment Distribution');
legend('Measurements', sprintf('Mean = %.6f m', mean_delta_d), 'Location', 'best');
grid on;

% Heading increment histogram
subplot(1, 2, 2);
histogram(measurements(2, :), 20, 'FaceColor', [0.8 0.4 0.2]);
hold on;
xline(mean_delta_beta, '--r', 'LineWidth', 2);
xlabel('Heading Increment Δβ [rad]');
ylabel('Frequency');
title('Heading Increment Distribution');
legend('Measurements', sprintf('Mean = %.6f rad', mean_delta_beta), 'Location', 'best');
grid on;

%% Reference Values (for comparison)
% Linear motion (v=0.1, ω=0):  Δd ≈ 0.002 m,  σ_d ≈ 9.5e-05 m
% Rotation (v=0, ω=0.1):       Δβ ≈ 0.002 rad, σ_β ≈ 3.9e-05 rad
