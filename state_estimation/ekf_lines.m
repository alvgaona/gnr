clear;
close all;
clc;
rng(0);

set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');

%% Map Definition

% Map is defined in Hesse form: (d,α).
% d: perpendicular distance to the origin.
% α: angle of that vector whose norm is the perpendicular distance.
hesse_map = HesseMap();

% Add horizontal and vertical lines
hesse_map.addLine(deg2rad(90), 0);
hesse_map.addLine(deg2rad(90), 10);
hesse_map.addLine(deg2rad(0), 0);
hesse_map.addLine(deg2rad(0), 10);

num_walls = hesse_map.getNumLines();

map_lines = hesse_map.getLines();

%% Vehicle and Simulation Parameters
dt = 0.1;                          % Discrete time step [s] - 10 Hz
sim_time = 100;                    % Total simulation time [s]
num_steps = sim_time / dt;         % Total simulation steps

%% Process Noise (continuous-time)
process_noise_velocity = 0.05;       % Linear velocity noise [m/s/sqrt(s)]
process_noise_yaw_rate = deg2rad(2); % Yaw rate noise [rad/s/sqrt(s)]

%% Measurement Noise
measurement_noise_range = 0.01;      % Range measurement noise [m]
max_range = 8;                       % Maximum sensor range [m]

%% Ground-truth Trajectory
true_trajectory = zeros(3, num_steps);      % [x; y; theta]
control_history = zeros(2, num_steps);      % [v; omega]
true_trajectory(:, 1) = [1; 1; 0];          % Start at (1,1) heading east

nominal_velocity = 0.5; % Nominal speed [m/s]
for k = 1:num_steps-1
    if mod(k, 100) < 75
        control_history(:, k) = [nominal_velocity; 0];
    else
        control_history(:, k) = [nominal_velocity; deg2rad(30)];
    end

    v = control_history(1, k);
    omega = control_history(2, k);

    % Integrate using Euler method (we could use more accurate methods)
    theta_k = true_trajectory(3, k);
    true_trajectory(:, k+1) = [
        true_trajectory(1, k) + v * dt * cos(theta_k);
        true_trajectory(2, k) + v * dt * sin(theta_k);
        theta_k + omega * dt
    ];
end

%% Dead Reckoning (open-loop)
dead_reckoning_trajectory = zeros(3, num_steps);
dead_reckoning_trajectory(:, 1) = true_trajectory(:, 1);

% Noise parameters for random walk drift
drift_noise_x = 0.005;            % Position drift diffusion [m/sqrt(s)]
drift_noise_y = 0.005;            % Position drift diffusion [m/sqrt(s)]
drift_noise_theta = deg2rad(0.2); % Heading drift diffusion [rad/sqrt(s)]

% Initialize accumulated drift
accumulated_drift = [0; 0; 0];

for k = 1:num_steps-1
    accumulated_drift = accumulated_drift + [
        drift_noise_x * randn * sqrt(dt);
        drift_noise_y * randn * sqrt(dt);
        drift_noise_theta * randn * sqrt(dt)
    ];

    v = control_history(1, k);
    omega = control_history(2, k);

    theta_k = dead_reckoning_trajectory(3, k);
    propagated_state = dead_reckoning_trajectory(:, k) + [
        v * dt * cos(theta_k);
        v * dt * sin(theta_k);
        omega * dt
    ];

    dead_reckoning_trajectory(:, k+1) = propagated_state + accumulated_drift;
end

%% EKF Initialization
x = dead_reckoning_trajectory(:, 1);  % Initial state
P = diag([0.1 0.1 deg2rad(1)].^2);    % Initial state covariance

% Process noise covariance for odometry [Δd, Δβ]
process_noise_d = process_noise_velocity;      % Distance increment noise [m]
process_noise_beta = process_noise_yaw_rate;   % Heading increment noise [rad]
Q = diag([process_noise_d^2, process_noise_beta^2]);

% Create EKF instance
chi2_threshold = 9.21;  % 99% confidence for 2 DOF
ekf = EKFLines(x, P, map_lines, Q, chi2_threshold);

estimated_trajectory = zeros(3, num_steps);
estimated_trajectory(:, 1) = x;

% Storage for covariance history
covariance_history = zeros(3, num_steps);
covariance_history(:, 1) = sqrt(diag(P));

%% Main EKF Loop
for k = 2:num_steps
    % Simulate odometry sensors
    state_delta = true_trajectory(:, k) - true_trajectory(:, k-1);
    actual_delta_d = norm(state_delta(1:2));
    actual_delta_theta = state_delta(3);

    % Add odometry measurement noise
    noisy_delta_d = actual_delta_d + process_noise_d * sqrt(dt) * randn;
    noisy_delta_theta = actual_delta_theta + process_noise_beta * sqrt(dt) * randn;
    
    % EKF Prediction with noisy odometry [Δd, Δβ]
    ekf.predict(noisy_delta_d, noisy_delta_theta);

    % EKF Update
    scan = lms_scan(true_trajectory(:, k), map_lines, ...
        max_range, measurement_noise_range, 'LMS100');
    
    % Lines are observed in the Hessse form
    lines_observed = ransac_lines(scan, 0.02, 5);  

    ekf.update(lines_observed);
    
    % Store variables of interest
    estimated_trajectory(:, k) = ekf.x;
    covariance_history(:, k) = sqrt(diag(ekf.P)); % Store standard deviations
end

%% Compute Estimation Errors
position_error_dr = sqrt(sum((dead_reckoning_trajectory(1:2, :) - true_trajectory(1:2, :)).^2, 1));
position_error_ekf = sqrt(sum((estimated_trajectory(1:2, :) - true_trajectory(1:2, :)).^2, 1));
time_vector = (0:num_steps-1) * dt;

%% Display Statistics
fprintf('\n========== Line-EKF with LMS200 Laser Scanner - Results ==========\n');
fprintf('Vehicle Model:          Unicycle [v, ω]\n');
fprintf('Measurement Model:      Line features from laser scan\n');
fprintf('Map:                    %d walls in Hesse form\n', num_walls);
fprintf('Time Step:              %.2f s (%.0f Hz)\n', dt, 1/dt);
fprintf('Simulation Time:        %.2f s\n', sim_time);
fprintf('Number of Steps:        %d\n', num_steps);
fprintf('\nProcess Noise:\n');
fprintf('  Velocity Std Dev:     %.3f m/s/sqrt(s)\n', process_noise_velocity);
fprintf('  Yaw Rate Std Dev:     %.3f rad/s/sqrt(s) (%.2f deg/s/sqrt(s))\n', ...
    process_noise_yaw_rate, rad2deg(process_noise_yaw_rate));
fprintf('\nMeasurement Noise:\n');
fprintf('  Range Std Dev:        %.3f m\n', measurement_noise_range);
fprintf('  Max Range:            %.1f m\n', max_range);
fprintf('\nDead Reckoning Performance:\n');
fprintf('  Final Position Error:   %.3f m\n', position_error_dr(end));
fprintf('  Mean Position Error:    %.3f m\n', mean(position_error_dr));
fprintf('  Max Position Error:     %.3f m\n', max(position_error_dr));
fprintf('\nLine-EKF Performance:\n');
fprintf('  Final Position Error:   %.3f m\n', position_error_ekf(end));
fprintf('  Mean Position Error:    %.3f m\n', mean(position_error_ekf));
fprintf('  Max Position Error:     %.3f m\n', max(position_error_ekf));
fprintf('===================================================================\n\n');

%% Visualization (ignore logic)
fig_animation = figure('Name', 'EKF Animation', 'Position', [50 50 1400 800]);

subplot(2, 2, [1 3]);
hold on; grid on; axis equal;

% Draw map walls once
for w = 1:num_walls
    alpha = map_lines(w, 1);
    d = map_lines(w, 2);
    n = [cos(alpha), sin(alpha)];
    p1 = d * n + 20 * [-n(2), n(1)];
    p2 = d * n - 20 * [-n(2), n(1)];
    plot([p1(1) p2(1)], [p1(2) p2(2)], 'k-', 'LineWidth', 2);
end

h_true_traj = plot(true_trajectory(1, 1), true_trajectory(2, 1), 'b-', 'LineWidth', 2);
h_dead_traj = plot(dead_reckoning_trajectory(1, 1), dead_reckoning_trajectory(2, 1), 'c--', 'LineWidth', 1.5);
h_est_traj = plot(estimated_trajectory(1, 1), estimated_trajectory(2, 1), 'r--', 'LineWidth', 2);
h_true_robot = plot(true_trajectory(1, 1), true_trajectory(2, 1), 'bo', 'MarkerSize', 12, 'MarkerFaceColor', 'b');
h_est_robot = plot(estimated_trajectory(1, 1), estimated_trajectory(2, 1), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
h_scan = plot(NaN, NaN, 'r.', 'MarkerSize', 4);
h_rays = plot(NaN, NaN, 'Color', [1 0.5 0.5], 'LineWidth', 0.5);  % Light red rays
h_observed_lines = [];  % Will hold multiple line handles

xlabel('X [m]'); ylabel('Y [m]');
title('Spatial View');

h_map = plot(NaN, NaN, 'k-', 'LineWidth', 2);
h_lidar_rays = plot(NaN, NaN, 'Color', [1 0.5 0.5], 'LineWidth', 0.5);
h_lidar_points = plot(NaN, NaN, 'r.', 'MarkerSize', 4);
h_observed = plot(NaN, NaN, 'g-', 'LineWidth', 2);

legend([h_map, h_true_traj, h_dead_traj, h_est_traj, h_lidar_rays, h_lidar_points, h_observed], ...
       {'Map Walls', 'True Trajectory', 'Dead Reckoning', 'EKF Estimate', ...
        'LiDAR Rays', 'LiDAR Points', 'Observed Lines'}, ...
       'Location', 'best', 'AutoUpdate', 'off');
xlim([-5 12]); ylim([-5 12]);

subplot(2, 2, 2);
hold on; grid on;
h_error_dr = plot(time_vector(1), position_error_dr(1), 'c-', 'LineWidth', 1.5);
h_error_ekf = plot(time_vector(1), position_error_ekf(1), 'r-', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Position Error [m]');
title('Position Error');
legend('Dead Reckoning', 'Line-EKF', 'Location', 'best', 'AutoUpdate', 'off');
xlim([0 sim_time]); ylim([0 max(max(position_error_dr), max(position_error_ekf)) * 1.1]);

subplot(2, 2, 4);
hold on; grid on;
h_cov_x = plot(time_vector(1), covariance_history(1, 1), 'r-', 'LineWidth', 1.5);
h_cov_y = plot(time_vector(1), covariance_history(2, 1), 'g-', 'LineWidth', 1.5);
h_cov_theta = plot(time_vector(1), rad2deg(covariance_history(3, 1)), 'b-', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Standard Deviation');
title('EKF Uncertainty ($\sigma$)');
legend('$\sigma_x$ [m]', '$\sigma_y$ [m]', '$\sigma_\theta$ [deg]', 'Location', 'best', 'AutoUpdate', 'off');
xlim([0 sim_time]);
ylim([0 max([max(covariance_history(1:2, :)), rad2deg(max(covariance_history(3, :)))]) * 1.1]);

animation_step = 5; 
for k = 1:animation_step:num_steps
    subplot(2, 2, [1 3]);

    set(h_true_traj, 'XData', true_trajectory(1, 1:k), 'YData', true_trajectory(2, 1:k));
    set(h_dead_traj, 'XData', dead_reckoning_trajectory(1, 1:k), 'YData', dead_reckoning_trajectory(2, 1:k));
    set(h_est_traj, 'XData', estimated_trajectory(1, 1:k), 'YData', estimated_trajectory(2, 1:k));

    set(h_true_robot, 'XData', true_trajectory(1, k), 'YData', true_trajectory(2, k));
    set(h_est_robot, 'XData', estimated_trajectory(1, k), 'YData', estimated_trajectory(2, k));

    scan = lms_scan(true_trajectory(:, k), map_lines, max_range, measurement_noise_range, 'LMS100');
    valid = ~isnan(scan(:, 1));

    if any(valid)
        theta_k = true_trajectory(3, k);
        robot_pos = true_trajectory(1:2, k);

        scan_xy = scan(valid, 1) .* [cos(scan(valid, 2) + theta_k), sin(scan(valid, 2) + theta_k)];
        scan_world = [robot_pos(1) + scan_xy(:, 1), robot_pos(2) + scan_xy(:, 2)];
        set(h_scan, 'XData', scan_world(:, 1), 'YData', scan_world(:, 2));

        n_rays = sum(valid);
        ray_x = zeros(3 * n_rays, 1);
        ray_y = zeros(3 * n_rays, 1);
        for i = 1:n_rays
            idx = 3 * (i - 1);
            ray_x(idx + 1) = robot_pos(1);
            ray_y(idx + 1) = robot_pos(2);
            ray_x(idx + 2) = scan_world(i, 1);
            ray_y(idx + 2) = scan_world(i, 2);
            ray_x(idx + 3) = NaN;  % Space between rays
            ray_y(idx + 3) = NaN;
        end
        set(h_rays, 'XData', ray_x, 'YData', ray_y);
    else
        set(h_scan, 'XData', NaN, 'YData', NaN);
        set(h_rays, 'XData', NaN, 'YData', NaN);
    end

    lines_observed = ransac_lines(scan, 0.02, 5);

    if ~isempty(h_observed_lines)
        delete(h_observed_lines(isvalid(h_observed_lines)));
    end
    h_observed_lines = [];

    theta_k = true_trajectory(3, k);
    robot_pos = true_trajectory(1:2, k);
    R = [cos(theta_k) -sin(theta_k); sin(theta_k) cos(theta_k)];

    for j = 1:size(lines_observed, 1)
        alpha_robot = lines_observed(j, 1);
        d_robot = lines_observed(j, 2);

        n_robot = [cos(alpha_robot); sin(alpha_robot)];
        tangent_robot = [-sin(alpha_robot); cos(alpha_robot)];

        center_robot = d_robot * n_robot;
        center_world = robot_pos + R * center_robot;
        tangent_world = R * tangent_robot;

        p1 = center_world + 3 * tangent_world;
        p2 = center_world - 3 * tangent_world;

        h = plot([p1(1) p2(1)], [p1(2) p2(2)], 'g-', 'LineWidth', 2);
        h_observed_lines = [h_observed_lines; h];
    end

    subplot(2, 2, 2);
    set(h_error_dr, 'XData', time_vector(1:k), 'YData', position_error_dr(1:k));
    set(h_error_ekf, 'XData', time_vector(1:k), 'YData', position_error_ekf(1:k));

    subplot(2, 2, 4);
    set(h_cov_x, 'XData', time_vector(1:k), 'YData', covariance_history(1, 1:k));
    set(h_cov_y, 'XData', time_vector(1:k), 'YData', covariance_history(2, 1:k));
    set(h_cov_theta, 'XData', time_vector(1:k), 'YData', rad2deg(covariance_history(3, 1:k)));

    drawnow('limitrate');
    pause(0.05);
end

figure(fig_animation);
fprintf('Animation complete!\n');
