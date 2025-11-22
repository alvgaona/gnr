clear;
close all;
clc;
rng(0);  % Repeatable random number generation

%% Map Definition

% Lines in Hesse normal form from the world/odom/map frame
map_lines = [
    deg2rad(90) 0; deg2rad(90)  10;  % Horizontal walls
    deg2rad(0) 0; deg2rad(0)  10     % Vertical walls
];

% Add two internal walls to create corners
map_lines = [
    map_lines;
    deg2rad(0) 4;
    deg2rad(90) 6
];
num_walls = size(map_lines, 1);

%% Vehicle and Simulation Parameters
nominal_velocity = 0.5;         % Nominal speed [m/s]
wheel_base = 0.4;               % Wheel-base (for bicycle model) [m]
time_step = 0.1;                % Discrete time step [s] - 10 Hz
num_steps = 400;                % Total simulation steps
simulation_time = num_steps * time_step;  % Total simulation time [s]

%% Process Noise (Motion Model Uncertainty)
% Velocity and yaw-rate noise in continuous time
process_noise_velocity = 0.05;       % Linear velocity noise [m/s/sqrt(s)]
process_noise_yaw_rate = deg2rad(2); % Yaw rate noise [rad/s/sqrt(s)]

%% Measurement Noise (Sensor Uncertainty)
% LMS200 laser scanner parameters
measurement_noise_range = 0.01;      % Range measurement noise [m]
max_range = 8;                       % Maximum sensor range [m]

%% Initial Conditions

% Ground truth trajectory storage
true_trajectory = zeros(3, num_steps);      % [x; y; theta]
control_history = zeros(2, num_steps);      % [v; omega]
true_trajectory(:, 1) = [1; 1; 0];          % Start at (1,1) heading east

%% Generate Ground Truth Trajectory (Bicycle Model)
for k = 1:num_steps-1
    % Simple "square" command pattern: drive straight, then turn
    if mod(k, 100) < 75
        control_history(:, k) = [nominal_velocity; 0];
    else
        control_history(:, k) = [nominal_velocity; deg2rad(30)];
    end

    v = control_history(1, k);
    omega = control_history(2, k);

    % Integrate using Euler method
    theta_k = true_trajectory(3, k);
    true_trajectory(:, k+1) = [
        true_trajectory(1, k) + v * time_step * cos(theta_k);
        true_trajectory(2, k) + v * time_step * sin(theta_k);
        theta_k + omega * time_step
    ];
end

%% Dead Reckoning (Open Loop)
% Accumulates drift via random walk - errors compound over time
dead_reckoning_trajectory = zeros(3, num_steps);
dead_reckoning_trajectory(:, 1) = true_trajectory(:, 1);

% Noise parameters for random walk drift (continuous-time diffusion)
drift_noise_x = 0.005;            % Position drift diffusion [m/sqrt(s)]
drift_noise_y = 0.005;            % Position drift diffusion [m/sqrt(s)]
drift_noise_theta = deg2rad(0.2); % Heading drift diffusion [rad/sqrt(s)]

% Initialize accumulated drift (starts at zero, grows as random walk)
accumulated_drift = [0; 0; 0];

for k = 1:num_steps-1
    % Random walk: accumulate drift proportional to sqrt(time_step)
    accumulated_drift = accumulated_drift + [
        drift_noise_x * randn * sqrt(time_step);
        drift_noise_y * randn * sqrt(time_step);
        drift_noise_theta * randn * sqrt(time_step)
    ];

    v = control_history(1, k);
    omega = control_history(2, k);

    theta_k = dead_reckoning_trajectory(3, k);
    propagated_state = dead_reckoning_trajectory(:, k) + [
        v * time_step * cos(theta_k);
        v * time_step * sin(theta_k);
        omega * time_step
    ];

    % Add accumulated drift to get dead reckoning estimate
    dead_reckoning_trajectory(:, k+1) = propagated_state + accumulated_drift;
end

%% EKF Initialization
estimated_state = dead_reckoning_trajectory(:, 1);  % Start from same initial state
P = diag([0.1 0.1 deg2rad(1)].^2);                  % Initial state covariance

% Process noise covariance for odometry [Δd, Δβ]
% Discretized from continuous-time: Q_discrete = Q_continuous * dt
process_noise_d = process_noise_velocity * sqrt(time_step);      % Distance increment noise [m]
process_noise_beta = process_noise_yaw_rate * sqrt(time_step);   % Heading increment noise [rad]
Q = diag([process_noise_d^2, process_noise_beta^2]);

% Storage for plotting
estimated_trajectory = zeros(3, num_steps);
estimated_trajectory(:, 1) = estimated_state;

%% Main EKF Loop
for k = 2:num_steps
    % Simulate odometry sensors
    state_delta = true_trajectory(:, k) - true_trajectory(:, k-1);
    actual_delta_d = norm(state_delta(1:2));
    actual_delta_theta = state_delta(3);

    % Add odometry measurement noise
    noisy_delta_d = actual_delta_d + process_noise_velocity * sqrt(time_step) * randn;
    noisy_delta_theta = actual_delta_theta + process_noise_yaw_rate * sqrt(time_step) * randn;

    %% EKF PREDICTION STEP
    theta_k = estimated_state(3);

    % Midpoint odometry model prediction
    theta_mid = theta_k + noisy_delta_theta / 2;
    predicted_state = [
        estimated_state(1) + noisy_delta_d * cos(theta_mid);
        estimated_state(2) + noisy_delta_d * sin(theta_mid);
        theta_k + noisy_delta_theta
    ];

    % Compute Jacobian of discrete transition function with respect to state
    % f(x,y,θ) = [x + Δd*cos(θ + Δβ/2); y + Δd*sin(θ + Δβ/2); θ + Δβ]
    A = [
        1, 0, -noisy_delta_d * sin(theta_mid);
        0, 1,  noisy_delta_d * cos(theta_mid);
        0, 0,  1
    ];

    % Compute Jacobian with respect to odometry input u = [Δd, Δβ]
    % For midpoint model: ∂f/∂[Δd, Δβ]
    W = [
        cos(theta_mid),  -0.5 * noisy_delta_d * sin(theta_mid);
        sin(theta_mid),   0.5 * noisy_delta_d * cos(theta_mid);
        0,                1
    ];

    % Predict covariance
    P_predicted = A * P * A' + W * Q * W';

    %% Generate LMS200 Laser Scan (from true pose)
    scan = simulateLaserScan(true_trajectory(:, k), map_lines, max_range, measurement_noise_range);

    %% Extract Line Features using RANSAC
    lines_observed = extractLinesRANSAC(scan, 0.02, 5);  % (alpha, d) + covariances

    %% EKF UPDATE STEP - Process Each Observed Line
    for j = 1:size(lines_observed, 1)
        % Based on the lines detected and extracted from the LMS200
        % sensor and RANSAC, the lines are based on the robot frame.
        alpha_observed = lines_observed(j, 1);  % Angle of line normal (robot frame)
        d_observed = lines_observed(j, 2);      % Distance to line (robot frame)
        sigma_alpha = lines_observed(j, 3);     % Angular uncertainty
        sigma_d = lines_observed(j, 4);         % Distance uncertainty

        % Data association should be performed.
        % Basically, try to match the real lines with the ones we observe
        % from the robot itself. Since the real lines are parameterized
        % in the world frame, we should apply a frame transformation.
        theta_predicted = predicted_state(3);
        alpha_world = alpha_observed + theta_predicted;

        % The matching happens by finding the closest matching line
        % for each observed line.
        angle_differences = zeros(num_walls, 1);
        for w = 1:num_walls
            angle_differences(w) = abs(normalizeAngle(map_lines(w, 1), alpha_world));
        end
        [~, idx] = min(angle_differences);
        alpha_map = map_lines(idx, 1);  % Map line angle (world frame)
        d_map = map_lines(idx, 2);      % Map line distance from origin

        % Predicted measurement: What we'd observe if at predicted_state
        % Measurement model: h(x) = [alpha_map - theta; d_map - n'*[x;y]]
        normal_map = [cos(alpha_map); sin(alpha_map)];
        alpha_predicted = alpha_map - theta_predicted;  % Expected angle in robot frame
        d_predicted = d_map - normal_map' * predicted_state(1:2);  % Expected distance

        % Innovation (measurement residual)
        innovation_angle = normalizeAngle(alpha_observed, alpha_predicted);
        innovation_distance = d_observed - d_predicted;
        innovation = [innovation_angle; innovation_distance];

        % Measurement Jacobian H (2x3 matrix)
        % h(x,y,θ) = [alpha_map - θ; d_map - cos(alpha_map)*x - sin(alpha_map)*y]
        H = [0,              0,              -1;
             -cos(alpha_map), -sin(alpha_map), 0];

        % Measurement noise covariance
        R = diag([sigma_alpha^2, sigma_d^2]);

        % Kalman update equations
        S = H * P_predicted * H' + R;           % Innovation covariance
        K = P_predicted * H' / S;               % Kalman gain
        predicted_state = predicted_state + K * innovation;
        P_predicted = (eye(3) - K * H) * P_predicted;
    end

    %% Store Results
    estimated_state = predicted_state;
    P = P_predicted;
    estimated_trajectory(:, k) = estimated_state;
end

%% Compute Estimation Errors
position_error_dr = sqrt(sum((dead_reckoning_trajectory(1:2, :) - true_trajectory(1:2, :)).^2, 1));
position_error_ekf = sqrt(sum((estimated_trajectory(1:2, :) - true_trajectory(1:2, :)).^2, 1));
time_vector = (0:num_steps-1) * time_step;

%% Visualization
figure('Name', 'Line-EKF with LMS200 Laser Scanner', 'Position', [50 50 1400 900]);

% Plot 1: 2D Trajectory with Map
subplot(2, 3, 1);
hold on; grid on; axis equal;
% Draw map walls
for w = 1:num_walls
    alpha = map_lines(w, 1);
    d = map_lines(w, 2);
    n = [cos(alpha), sin(alpha)];
    % Two points on the line (±20m along the line)
    p1 = d * n + 20 * [-n(2), n(1)];
    p2 = d * n - 20 * [-n(2), n(1)];
    plot([p1(1) p2(1)], [p1(2) p2(2)], 'k-', 'LineWidth', 1.5);
end
plot(true_trajectory(1, :), true_trajectory(2, :), 'b-', 'LineWidth', 2, 'DisplayName', 'True');
plot(dead_reckoning_trajectory(1, :), dead_reckoning_trajectory(2, :), 'c--', 'LineWidth', 1.5, 'DisplayName', 'Dead Reckoning');
plot(estimated_trajectory(1, :), estimated_trajectory(2, :), 'r--', 'LineWidth', 2, 'DisplayName', 'Line-EKF');
plot(true_trajectory(1, 1), true_trajectory(2, 1), 'go', 'MarkerSize', 12, 'MarkerFaceColor', 'g');
xlabel('X [m]'); ylabel('Y [m]');
title('Trajectory with Map Walls');
legend('Ground truth', 'Dead Reckoning', 'Line-EKF', 'Start', 'Location', 'best');

% Plot 2: X Position
subplot(2, 3, 2);
plot(time_vector, true_trajectory(1, :), 'b-', 'LineWidth', 1.5); hold on;
plot(time_vector, dead_reckoning_trajectory(1, :), 'c--', 'LineWidth', 1.5);
plot(time_vector, estimated_trajectory(1, :), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('X [m]');
title('X Position');
legend('Ground truth', 'Dead Reckoning', 'Line-EKF');
grid on;

% Plot 3: Y Position
subplot(2, 3, 3);
plot(time_vector, true_trajectory(2, :), 'b-', 'LineWidth', 1.5); hold on;
plot(time_vector, dead_reckoning_trajectory(2, :), 'c--', 'LineWidth', 1.5);
plot(time_vector, estimated_trajectory(2, :), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Y [m]');
title('Y Position');
legend('Ground truth', 'Dead Reckoning', 'Line-EKF');
grid on;

% Plot 4: Heading Angle
subplot(2, 3, 4);
plot(time_vector, rad2deg(true_trajectory(3, :)), 'b-', 'LineWidth', 1.5); hold on;
plot(time_vector, rad2deg(dead_reckoning_trajectory(3, :)), 'c--', 'LineWidth', 1.5);
plot(time_vector, rad2deg(estimated_trajectory(3, :)), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]'); ylabel('Heading [deg]');
title('Heading Angle');
legend('Ground truth', 'Dead Reckoning', 'Line-EKF');
grid on;

% Plot 5: Position Error
subplot(2, 3, 5);
plot(time_vector, position_error_dr, 'c-', 'LineWidth', 1.5); hold on;
plot(time_vector, position_error_ekf, 'r-', 'LineWidth', 2);
xlabel('Time [s]'); ylabel('Position Error [m]');
title('Position Estimation Error');
legend('Dead Reckoning', 'Line-EKF');
grid on;

% Plot 6: State Uncertainty (Covariance)
subplot(2, 3, 6);
% Note: variance_history not stored in original, so we can't plot it
% Just show a placeholder
text(0.5, 0.5, 'Covariance history not stored', 'HorizontalAlignment', 'center');
axis off;

%% Visualize Map with Laser Scans
figure('Name', 'Map with Laser Scan Overlay', 'Color', 'w');
hold on; grid on; axis equal;

% Draw map walls
for w = 1:num_walls
    alpha = map_lines(w, 1);
    d = map_lines(w, 2);
    n = [cos(alpha), sin(alpha)];
    p1 = d * n + 20 * [-n(2), n(1)];
    p2 = d * n - 20 * [-n(2), n(1)];
    plot([p1(1) p2(1)], [p1(2) p2(2)], 'k-', 'LineWidth', 2);
end

% Overlay laser scans every 20 steps
plot(true_trajectory(1, 1), true_trajectory(2, 1), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
for k = 1:20:num_steps
    scan = simulateLaserScan(true_trajectory(:, k), map_lines, max_range, measurement_noise_range);
    valid = ~isnan(scan(:, 1));
    xy = scan(valid, 1) .* [cos(scan(valid, 2) + true_trajectory(3, k)), ...
                            sin(scan(valid, 2) + true_trajectory(3, k))];
    plot(true_trajectory(1, k) + xy(:, 1), true_trajectory(2, k) + xy(:, 2), '.', 'Color', 0.7*[1 0 0]);
    plot(true_trajectory(1, k), true_trajectory(2, k), 'bo', 'MarkerFaceColor', 'b');
end

title('Map + Laser Returns (every 20 steps)');
xlabel('X [m]'); ylabel('Y [m]');
legend('Walls', 'Start', 'Scan points', 'Robot', 'Location', 'NorthOutside');

%% Display Statistics
fprintf('\n========== Line-EKF with LMS200 Laser Scanner - Results ==========\n');
fprintf('Vehicle Model:          Bicycle/Unicycle [v, ω]\n');
fprintf('Measurement Model:      Line features from laser scan\n');
fprintf('Map:                    %d walls in Hesse form\n', num_walls);
fprintf('Wheel Base (L):         %.2f m\n', wheel_base);
fprintf('Time Step:              %.2f s (%.0f Hz)\n', time_step, 1/time_step);
fprintf('Simulation Time:        %.2f s\n', simulation_time);
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

%% Support Functions

function scan = simulateLaserScan(robot_pose, map_lines, max_range, noise_std)
    % Simulates LMS200 laser scanner
    % Returns N×2 matrix [range(m), bearing(rad)]

    % LMS200: 180-degree field of view, 181 beams (1-degree resolution)
    angles = linspace(-pi/2, pi/2, 181);
    ranges = inf(size(angles));

    % For each wall, compute intersection with laser beams
    for w = 1:size(map_lines, 1)
        alpha = map_lines(w, 1);
        d_wall = map_lines(w, 2);
        n = [cos(alpha); sin(alpha)];

        % Check each laser beam for intersection
        for k = 1:numel(angles)
            phi = angles(k) + robot_pose(3);
            v = [cos(phi); sin(phi)];
            denom = n' * v;
            if abs(denom) > 1e-6
                t = (d_wall - n' * robot_pose(1:2)) / denom;
                if t > 0 && t < max_range
                    ranges(k) = min(ranges(k), t);
                end
            end
        end
    end

    % Mark out-of-range readings as NaN
    ranges(isinf(ranges)) = NaN;

    % Add measurement noise
    ranges = ranges + noise_std * randn(size(ranges));

    % Return [range, bearing] pairs
    scan = [ranges(:), angles(:)];
end

function lines = extractLinesRANSAC(scan, distance_threshold, min_points)
    % Extract line features from laser scan using RANSAC
    % Returns [alpha, d, sigma_alpha, sigma_d] for each line

    % Convert polar scan to Cartesian points (in robot frame)
    pts = scan(:, 1) .* [cos(scan(:, 2)), sin(scan(:, 2))];
    valid = ~isnan(scan(:, 1));
    pts = pts(valid, :);
    lines = [];  % Will store [alpha, d, sigma_alpha, sigma_d]

    % Iteratively find lines until insufficient points remain
    while size(pts, 1) > min_points
        best_inlier_count = 0;
        best_model = [];

        % RANSAC iterations
        for iter = 1:30
            % Randomly sample 2 points
            idx = randi(size(pts, 1), 2, 1);
            p1 = pts(idx(1), :);
            p2 = pts(idx(2), :);

            % Skip if points are too close
            if norm(p2 - p1) < 0.05
                continue;
            end

            % Compute line through p1 and p2
            dp = p2 - p1;
            n = [-dp(2), dp(1)] / norm(dp);  % Normal to line
            d = n * p1';

            % Count inliers
            in = abs(n * pts' - d) < distance_threshold;
            nin = sum(in);

            % Update best model
            if nin > best_inlier_count
                best_inlier_count = nin;
                best_model = [atan2(n(2), n(1)), d];
            end
        end

        % Stop if not enough inliers found
        if best_inlier_count < min_points
            break;
        end

        % Refine line using Total Least Squares (TLS) on inliers
        inlier_points = pts(best_inlier_count, :);
        [~, ~, V] = svd(inlier_points - mean(inlier_points, 1), 'econ');

        % Check for rank deficiency
        if size(V, 2) < 2
            pts(best_inlier_count, :) = [];  % Discard and continue
            continue;
        end

        % Normal is the minor axis (last singular vector)
        normal_refined = V(:, 2)';
        d_refined = normal_refined * mean(inlier_points, 1)';
        alpha_refined = atan2(normal_refined(2), normal_refined(1));

        % Estimate uncertainties from residuals
        res = normal_refined * inlier_points' - d_refined;
        sigma_d = std(res);

        % Crude angular uncertainty estimate
        sigma_alpha = sigma_d / sqrt(max(sum((inlier_points * normal_refined' - d_refined).^2), eps));

        % Store line parameters
        lines(end+1, :) = [alpha_refined, d_refined, sigma_alpha, sigma_d];

        % Remove inliers from point cloud
        pts(best_inlier_count, :) = [];
    end
end

function diff_angle = normalizeAngle(a, b)
    % Normalize angle difference to [-pi, pi]
    diff_angle = mod(a - b + pi, 2*pi) - pi;
end
