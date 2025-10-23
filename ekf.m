%% Extended Kalman Filter with Ackermann Model
% This script simulates a robot with Ackermann steering while using an EKF
% to estimate its position based on bearing measurements to beacons.

clear all;
close all;
clc;

%% Vehicle and Simulation Parameters
L = 2.5;                    % Wheelbase length [m]
time_step = 0.1;            % Discrete time step [s]
simulation_time = 10;       % Total simulation time [s]
num_steps = simulation_time / time_step;

%% Define Trajectory
% You can change these to create different trajectories
trajectory_type = 'circle';  % Options: 'straight', 'circle', 'curve'

switch trajectory_type
    case 'straight'
        % Straight line trajectory
        velocity_profile = 2.0 * ones(1, num_steps);           % Constant 2 m/s
        steering_profile = 0.0 * ones(1, num_steps);           % Zero steering

    case 'circle'
        % Circular trajectory
        velocity_profile = 1.0 * ones(1, num_steps);           % Constant 1 m/s
        radius = 5;                                             % 5m radius
        steering_profile = atan(L/radius) * ones(1, num_steps); % Constant steering

    case 'curve'
        % Curved path with varying steering
        velocity_profile = 1.5 * ones(1, num_steps);           % Constant 1.5 m/s
        steering_profile = 0.1 * sin(2*pi*(0:num_steps-1)/num_steps); % Sinusoidal steering
end

%% Process Noise (Motion Model Uncertainty)
process_noise_v = 0.01;      % Velocity noise [m/s]
process_noise_phi = 0.02;    % Steering angle noise [rad]
Q = [process_noise_v^2, 0;
     0, process_noise_phi^2];

%% Measurement Noise (Sensor Uncertainty)
measurement_noise_std = 0.05;  % Bearing measurement noise [rad]
R = measurement_noise_std^2 * eye(3);  % 3 beacons

%% Beacon Positions
beacons = [
    4,  8;   % Beacon 1
    1,  1;   % Beacon 2
    11, 3    % Beacon 3
];
num_beacons = size(beacons, 1);

%% Initial Conditions
% True initial state
true_state = [0; 0; pi/4];  % [x, y, theta] - Start at origin, heading 45 degrees

% Initial estimate (with error to show EKF correction)
estimated_state = true_state + [0.5; 0.5; 0.1];  % Small initial error

% Initial state covariance
P = diag([0.5^2, 0.5^2, 0.1^2]);  % Initial uncertainty

%% Storage for Results
true_trajectory = zeros(3, num_steps);
estimated_trajectory = zeros(3, num_steps);
variance_history = zeros(3, num_steps);
control_history = zeros(2, num_steps);

%% Main Simulation Loop
for step = 1:num_steps
    %% Generate Control Input with Noise
    true_control = [velocity_profile(step); steering_profile(step)];
    noisy_control = true_control + sqrt(Q) * randn(2, 1);
    control_history(:, step) = noisy_control;

    %% Simulate True Robot Motion (Ackermann Model)
    % Discrete-time Ackermann update
    theta = true_state(3);
    v = true_control(1);
    phi = true_control(2);

    true_state = true_state + time_step * [
        v * cos(theta);
        v * sin(theta);
        (v / L) * tan(phi)
    ];

    % Add small process noise to true state
    true_state = true_state + sqrt(time_step) * [
        sqrt(Q(1,1)) * randn;
        sqrt(Q(1,1)) * randn;
        sqrt(Q(2,2)) * randn
    ];

    true_trajectory(:, step) = true_state;

    %% Generate Measurements (Bearing to each beacon)
    measurements = zeros(num_beacons, 1);
    for b = 1:num_beacons
        % True bearing angle with noise
        true_bearing = atan2(beacons(b,2) - true_state(2), beacons(b,1) - true_state(1));
        relative_bearing = true_bearing - true_state(3);
        measurements(b) = relative_bearing + sqrt(R(b,b)) * randn;

        % Normalize to [-pi, pi]
        measurements(b) = atan2(sin(measurements(b)), cos(measurements(b)));
    end

    %% EKF PREDICTION STEP
    % Predict state using Ackermann model with noisy control
    v_hat = noisy_control(1);
    phi_hat = noisy_control(2);
    theta_prev = estimated_state(3);

    % Predicted state (Euler integration)
    predicted_state = estimated_state + time_step * [
        v_hat * cos(theta_prev);
        v_hat * sin(theta_prev);
        (v_hat / L) * tan(phi_hat)
    ];

    % Compute Jacobian of motion model with respect to state (A matrix)
    A = eye(3) + time_step * [
        0, 0, -v_hat * sin(theta_prev);
        0, 0,  v_hat * cos(theta_prev);
        0, 0,  0
    ];

    % Compute Jacobian of motion model with respect to control (W matrix)
    W = time_step * [
        cos(theta_prev), 0;
        sin(theta_prev), 0;
        tan(phi_hat)/L,  v_hat/(L * cos(phi_hat)^2)
    ];

    % Predict covariance
    P_predicted = A * P * A' + W * Q * W';

    %% EKF UPDATE STEP
    % Predict measurements
    predicted_measurements = zeros(num_beacons, 1);
    H = zeros(num_beacons, 3);  % Measurement Jacobian

    for b = 1:num_beacons
        dx = beacons(b,1) - predicted_state(1);
        dy = beacons(b,2) - predicted_state(2);
        dist_sq = dx^2 + dy^2;

        % Predicted bearing
        predicted_bearing = atan2(dy, dx);
        predicted_measurements(b) = predicted_bearing - predicted_state(3);

        % Normalize to [-pi, pi]
        predicted_measurements(b) = atan2(sin(predicted_measurements(b)), ...
                                         cos(predicted_measurements(b)));

        % Jacobian row for this beacon
        H(b, :) = [dy/dist_sq, -dx/dist_sq, -1];
    end

    % Innovation (measurement residual)
    innovation = measurements - predicted_measurements;

    % Normalize innovation angles to [-pi, pi]
    for b = 1:num_beacons
        innovation(b) = atan2(sin(innovation(b)), cos(innovation(b)));
    end

    % Innovation covariance
    S = H * P_predicted * H' + R;

    % Kalman gain
    K = P_predicted * H' / S;

    % Update state estimate
    estimated_state = predicted_state + K * innovation;

    % Normalize theta to [-pi, pi]
    estimated_state(3) = atan2(sin(estimated_state(3)), cos(estimated_state(3)));

    % Update covariance
    P = (eye(3) - K * H) * P_predicted;

    %% Store Results
    estimated_trajectory(:, step) = estimated_state;
    variance_history(:, step) = [P(1,1); P(2,2); P(3,3)];
end

%% Compute Estimation Errors
position_error = sqrt(sum((true_trajectory(1:2,:) - estimated_trajectory(1:2,:)).^2, 1));
angle_error = abs(true_trajectory(3,:) - estimated_trajectory(3,:));

%% Visualization
figure('Name', 'EKF with Ackermann Model', 'Position', [50 50 1400 900]);

% Plot 1: 2D Trajectory with Beacons
subplot(2,3,1);
hold on; grid on; axis equal;
plot(true_trajectory(1,:), true_trajectory(2,:), 'b-', 'LineWidth', 2, 'DisplayName', 'True');
plot(estimated_trajectory(1,:), estimated_trajectory(2,:), 'r--', 'LineWidth', 2, 'DisplayName', 'Estimated');
plot(true_trajectory(1,1), true_trajectory(2,1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
plot(beacons(:,1), beacons(:,2), 'ks', 'MarkerSize', 15, 'MarkerFaceColor', 'k', 'DisplayName', 'Beacons');
xlabel('X [m]'); ylabel('Y [m]');
title(sprintf('Trajectory (%s)', trajectory_type));
legend('Location', 'best');

% Plot 2: X Position
subplot(2,3,2);
plot(true_trajectory(1,:), 'b-', 'LineWidth', 1.5); hold on;
plot(estimated_trajectory(1,:), 'r--', 'LineWidth', 1.5);
xlabel('Time Step'); ylabel('X [m]');
title('X Position'); legend('True', 'Estimated');
grid on;

% Plot 3: Y Position
subplot(2,3,3);
plot(true_trajectory(2,:), 'b-', 'LineWidth', 1.5); hold on;
plot(estimated_trajectory(2,:), 'r--', 'LineWidth', 1.5);
xlabel('Time Step'); ylabel('Y [m]');
title('Y Position'); legend('True', 'Estimated');
grid on;

% Plot 4: Heading Angle
subplot(2,3,4);
plot(rad2deg(true_trajectory(3,:)), 'b-', 'LineWidth', 1.5); hold on;
plot(rad2deg(estimated_trajectory(3,:)), 'r--', 'LineWidth', 1.5);
xlabel('Time Step'); ylabel('Heading [deg]');
title('Heading Angle'); legend('True', 'Estimated');
grid on;

% Plot 5: Position Error
subplot(2,3,5);
plot(position_error, 'm-', 'LineWidth', 2);
xlabel('Time Step'); ylabel('Position Error [m]');
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

%% Control Input Visualization
figure('Name', 'Control Inputs', 'Position', [100 100 1000 500]);

subplot(2,1,1);
plot(control_history(1,:), 'b-', 'LineWidth', 1.5);
hold on; plot(velocity_profile, 'r--', 'LineWidth', 1);
xlabel('Time Step'); ylabel('Velocity [m/s]');
title('Velocity Control Input');
legend('Noisy', 'Commanded'); grid on;

subplot(2,1,2);
plot(rad2deg(control_history(2,:)), 'b-', 'LineWidth', 1.5);
hold on; plot(rad2deg(steering_profile), 'r--', 'LineWidth', 1);
xlabel('Time Step'); ylabel('Steering Angle [deg]');
title('Steering Angle Control Input');
legend('Noisy', 'Commanded'); grid on;

%% Display Statistics
fprintf('\n========== EKF with Ackermann Model - Results ==========\n');
fprintf('Trajectory Type:        %s\n', trajectory_type);
fprintf('Wheelbase (L):          %.2f m\n', L);
fprintf('Time Step:              %.2f s\n', time_step);
fprintf('Simulation Time:        %.2f s\n', simulation_time);
fprintf('Number of Steps:        %d\n', num_steps);
fprintf('\nFinal Position Error:   %.3f m\n', position_error(end));
fprintf('Mean Position Error:    %.3f m\n', mean(position_error));
fprintf('Max Position Error:     %.3f m\n', max(position_error));
fprintf('Final Angle Error:      %.3f deg\n', rad2deg(angle_error(end)));
fprintf('\nFinal Std Dev (x):      %.4f m\n', sqrt(variance_history(1,end)));
fprintf('Final Std Dev (y):      %.4f m\n', sqrt(variance_history(2,end)));
fprintf('Final Std Dev (theta):  %.4f rad\n', sqrt(variance_history(3,end)));
fprintf('=======================================================\n\n');
