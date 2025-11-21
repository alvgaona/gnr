%% LMS Laser Sensor Calibration Script
% Calibrates LMS laser range sensors by measuring statistical properties of
% raw range measurements from the center beam.
%
% Supports:
%   - LMS100: 270° FOV, 541 beams
%   - LMS200: 180° FOV, 181 beams
%
% This script:
%   1. Collects N raw laser measurements
%   2. Extracts center beam (forward-facing) range
%   3. Computes mean and standard deviation
%   4. Displays statistics and visualization
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
sensor_name = 'LMS200';         % Laser sensor name ('LMS100' or 'LMS200')
num_measurements = 100;         % Number of samples to collect

%% Initialize

% Storage for measurements
center_beam_measurements = zeros(1, num_measurements);

%% Collect Calibration Data

fprintf('Collecting %d laser measurements from %s...\n', num_measurements, sensor_name);

for i = 1:num_measurements
    % Get raw laser data
    laser_data = apoloGetLaserData(sensor_name);

    % Handle different sensor types
    if size(laser_data, 2) > 181
        % LMS100: 270° FOV, 541 beams
        % No special processing needed
    else
        % LMS200: 180° FOV, 181 beams
        laser_data = laser_data(1:180);  % Last measurement is often invalid
    end

    % Get center beam (forward-facing)
    center_index = round(size(laser_data, 2) / 2);
    center_beam_measurements(i) = laser_data(center_index);

    apoloUpdate();
end

fprintf('Data collection complete.\n\n');

%% Compute Statistics

mean_range = mean(center_beam_measurements);
std_range = std(center_beam_measurements);

%% Display Results

fprintf('========== LMS Calibration Results ==========\n');
fprintf('Sensor:                %s\n', sensor_name);
fprintf('Number of samples:     %d\n', num_measurements);
fprintf('Number of beams:       %d\n\n', size(laser_data, 2));

fprintf('Center Beam Range:\n');
fprintf('  Mean:              %.6f m\n', mean_range);
fprintf('  Std Dev:           %.6f m\n', std_range);
fprintf('=============================================\n\n');

%% Visualization

figure('Name', sprintf('LMS Calibration Results - %s', sensor_name), ...
       'Position', [100 100 1200 500]);

% Range measurements over time
subplot(1, 2, 1);
plot(center_beam_measurements, 'b-', 'LineWidth', 1.5);
hold on;
yline(mean_range, '--r', 'LineWidth', 2);
xlabel('Sample Number');
ylabel('Range [m]');
title('Center Beam Measurements over Time');
legend('Measurements', sprintf('Mean = %.4f m', mean_range), 'Location', 'best');
grid on;

% Range distribution
subplot(1, 2, 2);
histogram(center_beam_measurements, 20, 'FaceColor', [0.2 0.6 0.8]);
hold on;
xline(mean_range, '--r', 'LineWidth', 2);
xlabel('Range [m]');
ylabel('Frequency');
title('Center Beam Measurement Distribution');
legend('Measurements', sprintf('Mean = %.4f m', mean_range), 'Location', 'best');
grid on;
