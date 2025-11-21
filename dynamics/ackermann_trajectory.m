%% Ackermann Model - Linear Trajectory Simulation
% This script simulates a vehicle with Ackermann steering moving in a
% straight line and visualizes the results.

clear all;
close all;
clc;

%% Vehicle Parameters
L = 2.5;  % Wheelbase length [m]

%% Initial Conditions
initial_state = [0; 0; pi/4];  % [x, y, theta] - Start at origin, heading 45 degrees

%% Control Input for Linear Motion
% For straight line motion:
% - Constant forward velocity
% - Zero steering angle
linear_velocity = 2.0;  % [m/s]
steering_angle = 0.0;   % [rad] - zero for straight line

control_input = [linear_velocity; steering_angle];

%% Simulation Parameters
t_start = 0;
t_end = 10;  % Simulate for 10 seconds
time_span = [t_start t_end];

%% ODE Integration
% Create anonymous function for ODE solver
ode_fun = @(t, x) ackermann(x, control_input, L);

% Solve the differential equation
[time, state] = ode45(ode_fun, time_span, initial_state);

% Extract state components
x_pos = state(:, 1);
y_pos = state(:, 2);
theta = state(:, 3);

%% Visualization
figure('Name', 'Ackermann Model - Linear Trajectory', 'Position', [100 100 1200 800]);

% Plot 1: 2D Trajectory
subplot(2, 3, [1 4]);
plot(x_pos, y_pos, 'b-', 'LineWidth', 2);
hold on;
% Plot start and end points
plot(x_pos(1), y_pos(1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g', 'DisplayName', 'Start');
plot(x_pos(end), y_pos(end), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r', 'DisplayName', 'End');

% Plot vehicle orientation at several points
num_arrows = 10;
arrow_indices = round(linspace(1, length(time), num_arrows));
arrow_length = 0.5;
for i = arrow_indices
    quiver(x_pos(i), y_pos(i), arrow_length*cos(theta(i)), arrow_length*sin(theta(i)), ...
           'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.5, 'AutoScale', 'off');
end

grid on;
axis equal;
xlabel('X Position [m]', 'FontSize', 12);
ylabel('Y Position [m]', 'FontSize', 12);
title('Vehicle Trajectory (Linear Motion)', 'FontSize', 14, 'FontWeight', 'bold');
legend('Trajectory', 'Start', 'End', 'Location', 'best');

% Plot 2: X Position vs Time
subplot(2, 3, 2);
plot(time, x_pos, 'b-', 'LineWidth', 2);
grid on;
xlabel('Time [s]', 'FontSize', 11);
ylabel('X Position [m]', 'FontSize', 11);
title('X Position over Time', 'FontSize', 12);

% Plot 3: Y Position vs Time
subplot(2, 3, 3);
plot(time, y_pos, 'r-', 'LineWidth', 2);
grid on;
xlabel('Time [s]', 'FontSize', 11);
ylabel('Y Position [m]', 'FontSize', 11);
title('Y Position over Time', 'FontSize', 12);

% Plot 4: Heading Angle vs Time
subplot(2, 3, 5);
plot(time, rad2deg(theta), 'g-', 'LineWidth', 2);
grid on;
xlabel('Time [s]', 'FontSize', 11);
ylabel('Heading Angle [deg]', 'FontSize', 11);
title('Heading Angle over Time', 'FontSize', 12);

% Plot 5: Velocity Profile
subplot(2, 3, 6);
velocity_magnitude = sqrt(diff(x_pos).^2 + diff(y_pos).^2) ./ diff(time);
plot(time(1:end-1), velocity_magnitude, 'm-', 'LineWidth', 2);
hold on;
yline(linear_velocity, 'k--', 'Commanded Velocity', 'LineWidth', 1.5);
grid on;
xlabel('Time [s]', 'FontSize', 11);
ylabel('Velocity [m/s]', 'FontSize', 11);
title('Actual Velocity', 'FontSize', 12);
legend('Computed', 'Commanded', 'Location', 'best');

%% Display Summary Statistics
fprintf('\n========== Simulation Summary ==========\n');
fprintf('Wheelbase (L):          %.2f m\n', L);
fprintf('Linear Velocity:        %.2f m/s\n', linear_velocity);
fprintf('Steering Angle:         %.2f rad (%.2f deg)\n', steering_angle, rad2deg(steering_angle));
fprintf('Simulation Time:        %.2f s\n', t_end);
fprintf('Initial Position:       (%.2f, %.2f) m\n', x_pos(1), y_pos(1));
fprintf('Final Position:         (%.2f, %.2f) m\n', x_pos(end), y_pos(end));
fprintf('Initial Heading:        %.2f deg\n', rad2deg(theta(1)));
fprintf('Final Heading:          %.2f deg\n', rad2deg(theta(end)));
fprintf('Total Distance:         %.2f m\n', sum(sqrt(diff(x_pos).^2 + diff(y_pos).^2)));
fprintf('Expected Distance:      %.2f m\n', linear_velocity * t_end);
fprintf('=======================================\n\n');
