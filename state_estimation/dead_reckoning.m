clear; close all; clc; rng(0);

%% Simulation Parameters
dt = 0.1;           % Time step (s)
t_total = 20;       % Total simulation time (s)
t = 0:dt:t_total;   % Time vector

%% Control Inputs (velocity commands)
v = 1.0 * ones(size(t));   % Linear velocity (m/s)
omega = zeros(size(t));    % Angular velocity (rad/s)

% Add some turns
omega(t >= 5 & t < 8) = 0.3;      % Right turn
omega(t >= 12 & t < 15) = -0.4;   % Left turn

%% Ground truth
x_true = zeros(size(t));
y_true = zeros(size(t));
theta_true = zeros(size(t));

% Initial conditions
x_true(1) = 0;
y_true(1) = 0;
theta_true(1) = 0;

% Integrate using unicycle kinematics
for i = 2:length(t)
    theta_true(i) = theta_true(i-1) + omega(i-1) * dt;
    x_true(i) = x_true(i-1) + v(i-1) * cos(theta_true(i-1)) * dt;
    y_true(i) = y_true(i-1) + v(i-1) * sin(theta_true(i-1)) * dt;
end

%% Dead Reckoning with Odometry Errors
delta_d_drift_std = 0.05;       % Distance drift std (m/sqrt(s))
delta_theta_drift_std = 0.02;   % Angular drift std (rad/sqrt(s))

% Initialize drift accumulators (random walk)
delta_d_drift = 0;
delta_theta_drift = 0;

% Odometry positions (from applying v and omega with unicycle dynamics)
x = zeros(size(t));
y = zeros(size(t));
theta_odom = zeros(size(t));

% Dead reckoning estimate (using noisy odometry)
x_odom = zeros(size(t));
y_odom = zeros(size(t));
theta_dr = zeros(size(t));

% Initial conditions (same as ground truth)
x(1) = x_true(1);
y(1) = y_true(1);
theta_odom(1) = theta_true(1);
x_odom(1) = x_true(1);
y_odom(1) = y_true(1);
theta_dr(1) = theta_true(1);

% Dead reckoning with noisy odometry measurements
for i = 2:length(t)
    % Apply unicycle dynamics with clean control inputs to get odometry position
    theta_odom(i) = theta_odom(i-1) + omega(i-1) * dt;
    x(i) = x(i-1) + v(i-1) * cos(theta_odom(i-1)) * dt;
    y(i) = y(i-1) + v(i-1) * sin(theta_odom(i-1)) * dt;

    % Compute delta_d and delta_theta from odometry positions
    delta_d = sqrt((x(i) - x(i-1))^2 + (y(i) - y(i-1))^2);
    delta_theta = theta_odom(i) - theta_odom(i-1);

    % Update drift (random walk with sqrt(dt) scaling)
    delta_d_drift = delta_d_drift + delta_d_drift_std * sqrt(dt) * randn();
    delta_theta_drift = delta_theta_drift + delta_theta_drift_std * sqrt(dt) * randn();

    % Measured odometry with accumulated drift error
    delta_d_meas = delta_d + delta_d_drift;
    delta_theta_meas = delta_theta + delta_theta_drift;

    % Update dead reckoning estimate using midpoint dynamics
    theta_mid = theta_dr(i-1) + delta_theta_meas / 2;
    x_odom(i) = x_odom(i-1) + delta_d_meas * cos(theta_mid);
    y_odom(i) = y_odom(i-1) + delta_d_meas * sin(theta_mid);
    theta_dr(i) = theta_dr(i-1) + delta_theta_meas;
end

%% Calculate Position Error
pos_error = sqrt((x_true - x_odom).^2 + (y_true - y_odom).^2);
heading_error = rad2deg(abs(wrapToPi(theta_true - theta_dr)));

%% Plotting
set(0, 'DefaultTextInterpreter', 'latex');
set(0, 'DefaultLegendInterpreter', 'latex');

figure('Position', [100 100 1200 800]);

% Trajectory comparison
subplot(2,2,1);
plot(x_true, y_true, 'b-', 'LineWidth', 2); hold on;
plot(x, y, 'g-', 'LineWidth', 2);
plot(x_odom, y_odom, 'r--', 'LineWidth', 2);
plot(x_true(1), y_true(1), 'ko', 'MarkerSize', 10, 'LineWidth', 2);
plot(x_true(end), y_true(end), 'bs', 'MarkerSize', 10, 'LineWidth', 2);
plot(x(end), y(end), 'gs', 'MarkerSize', 10, 'LineWidth', 2);
plot(x_odom(end), y_odom(end), 'rs', 'MarkerSize', 10, 'LineWidth', 2);
grid on; axis equal;
xlabel('X Position (m)'); ylabel('Y Position (m)');
title('Trajectory Comparison');
legend('Ground Truth', 'Odometry (no drift)', 'Dead Reckoning (with drift)', ...
       'Start', 'True End', 'Odom End', 'DR End', 'Location', 'best');

% Position error over time
subplot(2,2,2);
plot(t, pos_error, 'r-', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Position Error (m)');
title('Position Error Growth');

% X and Y position comparison
subplot(2,2,3);
plot(t, x_true, 'b-', 'LineWidth', 2); hold on;
plot(t, x_odom, 'r--', 'LineWidth', 2);
plot(t, y_true, 'b-', 'LineWidth', 2);
plot(t, y_odom, 'r--', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Position (m)');
title('X and Y Positions vs Time');
legend('X True', 'X Odom', 'Y True', 'Y Odom', 'Location', 'best');

% Heading error over time
subplot(2,2,4);
plot(t, heading_error, 'm-', 'LineWidth', 2);
grid on;
xlabel('Time (s)'); ylabel('Heading Error (deg)');
title('Heading Error Growth');

% Print final statistics
fprintf('\n=== Dead Reckoning Error Statistics ===\n');
fprintf('Final position error: %.3f m\n', pos_error(end));
fprintf('Final heading error: %.2f deg\n', heading_error(end));
fprintf('Mean position error: %.3f m\n', mean(pos_error));
fprintf('Max position error: %.3f m\n', max(pos_error));
fprintf('Position drift rate: %.3f m per 10s\n', pos_error(end)/(t_total/10));

sgtitle('Dead Reckoning Estimation for a Unicycle Vehicle')
