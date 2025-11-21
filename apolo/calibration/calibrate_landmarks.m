%% Landmark Detection Calibration Script
% Calibrates landmark detection by measuring statistical properties of
% angle and distance measurements to detected landmarks.
%
% This script:
%   1. Collects N landmark measurements (angle and distance)
%   2. Computes mean and standard deviation for both measurements
%   3. Displays statistics and visualization
%
% Output:
%   - Displays calibration statistics
%   - Plots measurement distribution and time series
%
% Usage:
%   Modify parameters below, then run this script in MATLAB.

clear; clc;

%% Calibration Parameters

% Sensor configuration
sensor_name = 'LMS100';         % Laser sensor name ('LMS100' or 'LMS200')
num_measurements = 100;         % Number of samples to collect

%% Initialize

% Storage for measurements
angle_measurements = zeros(1, num_measurements);
distance_measurements = zeros(1, num_measurements);

%% Collect Calibration Data

fprintf('Collecting %d landmark measurements from %s...\n', num_measurements, sensor_name);

for i = 1:num_measurements
    % Get landmark detection data
    landmark_data = apoloGetLaserLandMarks(sensor_name);

    % Store angle and distance to first landmark
    angle_measurements(i) = landmark_data.angle(1);
    distance_measurements(i) = landmark_data.distance(1);

    apoloUpdate();
end

fprintf('Data collection complete.\n\n');

%% Compute Statistics

% Angle statistics
mean_angle = mean(angle_measurements);
std_angle = std(angle_measurements);

% Distance statistics
mean_distance = mean(distance_measurements);
std_distance = std(distance_measurements);

%% Display Results

fprintf('========== Landmark Calibration Results ==========\n');
fprintf('Sensor:                %s\n', sensor_name);
fprintf('Number of samples:     %d\n\n', num_measurements);

fprintf('Landmark Angle:\n');
fprintf('  Mean:              %.6f rad (%.4f deg)\n', ...
    mean_angle, rad2deg(mean_angle));
fprintf('  Std Dev:           %.6f rad (%.4f deg)\n\n', ...
    std_angle, rad2deg(std_angle));

fprintf('Landmark Distance:\n');
fprintf('  Mean:              %.6f m\n', mean_distance);
fprintf('  Std Dev:           %.6f m\n', std_distance);
fprintf('===================================================\n\n');

%% Visualization

figure('Name', 'Landmark Calibration Results', 'Position', [100 100 1400 600]);

% Angle measurements over time
subplot(2, 2, 1);
plot(angle_measurements, 'b-', 'LineWidth', 1.5);
hold on;
yline(mean_angle, '--r', 'LineWidth', 2);
xlabel('Sample Number');
ylabel('Angle [rad]');
title('Landmark Angle over Time');
legend('Measurements', sprintf('Mean = %.4f rad', mean_angle), 'Location', 'best');
grid on;

% Angle distribution
subplot(2, 2, 2);
histogram(angle_measurements, 20, 'FaceColor', [0.2 0.6 0.8]);
hold on;
xline(mean_angle, '--r', 'LineWidth', 2);
xlabel('Angle [rad]');
ylabel('Frequency');
title('Landmark Angle Distribution');
grid on;

% Distance measurements over time
subplot(2, 2, 3);
plot(distance_measurements, 'b-', 'LineWidth', 1.5);
hold on;
yline(mean_distance, '--r', 'LineWidth', 2);
xlabel('Sample Number');
ylabel('Distance [m]');
title('Landmark Distance over Time');
legend('Measurements', sprintf('Mean = %.4f m', mean_distance), 'Location', 'best');
grid on;

% Distance distribution
subplot(2, 2, 4);
histogram(distance_measurements, 20, 'FaceColor', [0.8 0.4 0.2]);
hold on;
xline(mean_distance, '--r', 'LineWidth', 2);
xlabel('Distance [m]');
ylabel('Frequency');
title('Landmark Distance Distribution');
grid on;
