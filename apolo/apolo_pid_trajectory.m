clear; close all; clc;

%% Simulation Parameters
WorldXML = readstruct("DafneEKFLong.xml","FileType","xml");
dt = 0.05;          % Time step (s)
t_end = 20;         % Simulation duration (s)
time = 0:dt:t_end;  % Time vector
robotName = convertStringsToChars(WorldXML.World.Pioneer3ATSim.nameAttribute);%LMS100Sim %LandMark mark_id="1"

%% Model Parameters
% State: [x,y,θ]
x = 0;              % Initial x position (m)
y = 0;              % Initial y position (m)
theta = 0;          % Initial heading angle (rad)

if apoloPlaceMRobot(robotName,[x,y,0],theta)~=1
    disp("Error placing "+robotName+" on position");
    return
end
loc = apoloGetLocationMRobot(robotName);%[x y z theta]
x=loc(1);y=loc(2);theta=loc(4);
apoloResetOdometry(robotName);
apoloUpdate();

% Control inputs: [v,ω]
v = 0;              % Linear velocity (m/s)
omega = 0;          % Angular velocity (rad/s)

%% PID Controller Parameters
Kp_v = 0.5;
Ki_v = 0.01;
Kd_v = 0.1;

Kp_omega = 2.0;
Ki_omega = 0.05;
Kd_omega = 0.3;

% Error terms initialization
error_dist_prev = 0;
error_dist_integral = 0;
error_heading_prev = 0;
error_heading_integral = 0;

% Maximum control inputs
v_max = 2.0;        % Maximum linear velocity (m/s)
omega_max = 2.0;    % Maximum angular velocity (rad/s)

%% Storage
trajectory = zeros(length(time), 3);  % Actual trajectory
desired_traj = zeros(length(time), 2); % Desired trajectory

%% Available trajectories
trajectory_type = 'circular';  % Options: 'circular', 'sinusoidal'

% Circular trajectory parameters
x_center = 5;
y_center = 5;
radius = 4;
angular_freq = 0.3; % Angular frequency for circular motion

% Sinusoidal trajectory parameters
forward_speed = 1.0;  % Forward speed along x-axis (m/s)
amplitude = 3.0;      % Amplitude of sine wave (m)
sine_freq = 0.4;      % Frequency of sine wave (rad/s)

switch trajectory_type
    case 'circular'
        % Circular trajectory
        desired_x = x_center + radius * cos(angular_freq * time);
        desired_y = y_center + radius * sin(angular_freq * time);

        % Desired velocities (derivative of position)
        desired_vx = -radius * angular_freq * sin(angular_freq * time);
        desired_vy = radius * angular_freq * cos(angular_freq * time);

    case 'sinusoidal'
        % Sinusoidal trajectory
        desired_x = forward_speed * time;
        desired_y = amplitude * sin(sine_freq * time);

        % Desired velocities (derivative of position)
        desired_vx = forward_speed * ones(size(time));
        desired_vy = amplitude * sine_freq * cos(sine_freq * time);

    otherwise
        error('Unknown trajectory type: %s', trajectory_type);
end

% Desired heading (tangent to the trajectory)
desired_theta = atan2(desired_vy, desired_vx);

% Store desired trajectory
desired_traj(:, 1) = desired_x;
desired_traj(:, 2) = desired_y;

%% Simulation Loop
for i = 1:length(time)
    % Position error
    error_x = desired_x(i) - x;
    error_y = desired_y(i) - y;

    % Distance error (magnitude)
    error_dist = sqrt(error_x^2 + error_y^2);

    % Heading error to target
    theta_target = atan2(error_y, error_x);
    error_heading = theta_target - theta;

    % Normalize angle error to [-pi, pi]
    error_heading = atan2(sin(error_heading), cos(error_heading));

    % PID for linear velocity (based on distance error)
    error_dist_derivative = (error_dist - error_dist_prev) / dt;
    error_dist_integral = error_dist_integral + error_dist * dt;

    v = Kp_v * error_dist + ...
        Ki_v * error_dist_integral + ...
        Kd_v * error_dist_derivative;

    % PID for angular velocity (based on heading error)
    error_heading_derivative = (error_heading - error_heading_prev) / dt;
    error_heading_integral = error_heading_integral + error_heading * dt;

    omega = Kp_omega * error_heading + ...
        Ki_omega * error_heading_integral + ...
        Kd_omega * error_heading_derivative;

    % Saturate control inputs
    v = max(min(v, v_max), 0);
    omega = max(min(omega, omega_max), -omega_max);

    % Store data
    trajectory(i, :) = [x, y, theta];

    % Update dynamics
    apoloMoveMRobot(robotName,[v omega],dt);
    apoloUpdate();
    pause(dt);
    loc = apoloGetLocationMRobot(robotName);%[x y z theta]
    x=loc(1);y=loc(2);theta=loc(4);

    % Normalize theta to [-pi, pi]
    theta = atan2(sin(theta), cos(theta));

    % Update previous errors
    error_dist_prev = error_dist;
    error_heading_prev = error_heading;
end

%% Plotting Results
figure('Position', [100, 100, 1200, 800]);

% Trajectory plot
subplot(2, 2, 1);
plot(desired_traj(:, 1), desired_traj(:, 2), 'b--', 'LineWidth', 2);
hold on;
plot(trajectory(:, 1), trajectory(:, 2), 'r-', 'LineWidth', 1.5);
plot(trajectory(1, 1), trajectory(1, 2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(trajectory(end, 1), trajectory(end, 2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('$x$ [m]', 'Interpreter', 'latex');
ylabel('$y$ [m]', 'Interpreter', 'latex');
title('Trajectory Tracking', 'Interpreter', 'latex');
legend('Desired', 'Actual', 'Start', 'End', 'Interpreter', 'latex');
grid on;
axis equal;

% Position errors over time
subplot(2, 2, 2);
error_x = desired_traj(:, 1) - trajectory(:, 1);
error_y = desired_traj(:, 2) - trajectory(:, 2);
tracking_error = sqrt(error_x.^2 + error_y.^2);
plot(time, tracking_error, 'b-', 'LineWidth', 1.5);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Tracking Error [m]', 'Interpreter', 'latex');
title('Position Tracking Error', 'Interpreter', 'latex');
grid on;

% X and Y positions over time
subplot(2, 2, 3);
plot(time, desired_traj(:, 1), 'b--', 'LineWidth', 2);
hold on;
plot(time, trajectory(:, 1), 'r-', 'LineWidth', 1.5);
plot(time, desired_traj(:, 2), 'g--', 'LineWidth', 2);
plot(time, trajectory(:, 2), 'm-', 'LineWidth', 1.5);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Position [m]', 'Interpreter', 'latex');
title('Position vs Time', 'Interpreter', 'latex');
legend('$x$ Desired', '$x$ Actual', '$y$ Desired', '$y$ Actual', 'Interpreter', 'latex');
grid on;

% Heading angle over time
subplot(2, 2, 4);
plot(time, rad2deg(trajectory(:, 3)), 'b-', 'LineWidth', 1.5);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Heading Angle $\theta$ (deg)', 'Interpreter', 'latex');
title('Heading Angle', 'Interpreter', 'latex');
grid on;

sgtitle(sprintf('Vehicle Trajectory Tracking with PID Controller - %s', trajectory_type), 'Interpreter', 'latex');
