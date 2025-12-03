clc; clear; close all;

%% Check CasADi availability
try
    import casadi.*
    fprintf('CasADi loaded successfully.\n');
catch
    error(['CasADi not found! Install from: https://web.casadi.org/get/\n', ...
           'Add CasADi to MATLAB path: addpath(''/path/to/casadi'')']);
end

%% Environment. Horizontal corridor with parallel walls
corridor_width = 4;  % Corridor width [m]
corridor_length = 12; % Corridor length [m]

% Define obstacles as cell array
obstacles = {};

% Corridor walls (finite line segments, now horizontal)
obstacles{end+1} = struct('type', 'wall', 'p1', [-1; corridor_width/2], 'p2', [corridor_length; corridor_width/2]);
obstacles{end+1} = struct('type', 'wall', 'p1', [-1; -corridor_width/2], 'p2', [corridor_length; -corridor_width/2]);

% Square obstacle in center of corridor
obstacle_center = [5; 0]; % Center position [x; y] (centered in corridor)
obstacle_size = 1.0;        % Square side length [m]
obstacles{end+1} = struct('type', 'square', 'center', obstacle_center, 'size', obstacle_size);

%% LiDAR Configuration
max_range = 10.0;       % Maximum sensor range [m]
noise_std = 0.02;       % Range measurement noise [m]
lidar_model = 'LMS200'; % 180-degree FOV, 181 beams

%% CBF Parameters
d_safe = 0.3;           % Safety radius [m]
alpha_cbf = 0.2;        % CBF aggressiveness parameter
scan_downsample = 30;   % Use every Nth LiDAR beam
constraint_range = 2.0; % Only use obstacles within this distance [m]
use_soft_cbf = true;    % Use soft CBF constraints to prevent infeasibility
cbf_penalty = 500;     % Penalty weight for CBF violations

%% NMPC Configuration
nx = 3;  % states: [x, y, theta]
nu = 2;  % inputs: [v, omega]

dt = 0.1;               % Sample time [s]
N = 20;                 % Prediction horizon
Q = diag([1, 1, 0.1]); % State tracking weights [x, y, theta]
R = diag([1, 1]);       % Control effort weights [v, omega]

% Progressive reference tracking (helps avoid getting stuck)
use_progressive_ref = true;  % Track predicted position, not global reference
progress_weight = 0.5;       % Weight for forward progress vs tracking

% Control constraints
v_min = 0;
v_max = 1.5;
omega_min = -2;
omega_max = 2;

%% Reference Trajectory
Tsim = 100;
t = 0:dt:Tsim;

% Straight horizontal trajectory through center of corridor
v_ref = 0.8;  % Reference speed [m/s]
x_ref = v_ref * t;           % Move horizontally at constant speed
y_ref = zeros(size(t));      % Stay centered in corridor
theta_ref = zeros(size(t));  % Point along x-axis (0 degrees)

xref = [x_ref', y_ref', theta_ref'];

%% Initial State
x0 = [0; 0; 0];  % Start at left of corridor, centered, pointing right

%% Build CasADi Opti problem (once, outside loop)
fprintf('Building CasADi Opti optimization problem...\n');

opti = casadi.Opti();

% Maximum number of obstacle points (define BEFORE using it)
n_obs_max = 10;

% Decision variables
U = opti.variable(nu, N);  % Control inputs over horizon
X = opti.variable(nx, N+1); % State trajectory

% Slack variables for soft CBF constraints (if enabled)
if use_soft_cbf
    Slack = opti.variable(n_obs_max, N);  % Slack for each obstacle at each timestep
    opti.subject_to(Slack(:) >= 0);  % Non-negative slack (vectorized)
end

% Parameters
X0 = opti.parameter(nx, 1);              % Initial state
Xref = opti.parameter(nx, N+1);          % Reference trajectory
Ranges = opti.parameter(n_obs_max, 1);   % LiDAR ranges
Bearings = opti.parameter(n_obs_max, 1); % LiDAR bearings
N_obs = opti.parameter(1, 1);            % Actual number of obstacles

% Objective
J = 0;
for k = 1:N
    % State tracking error
    x_err = X(:, k) - Xref(:, k);
    J = J + x_err' * Q * x_err;

    % Control effort
    J = J + U(:, k)' * R * U(:, k);

    % Penalty for CBF slack violations
    if use_soft_cbf
        J = J + cbf_penalty * sum(Slack(:, k).^2);
    end

    % Forward progress incentive (prevent getting stuck)
    if use_progressive_ref
        % Reward movement in positive x direction (corridor direction)
        J = J - progress_weight * U(1, k);  % Negative cost = reward velocity
    end
end

% Terminal cost
x_err = X(:, N+1) - Xref(:, N+1);
J = J + x_err' * Q * x_err;

opti.minimize(J);

% Dynamics constraints
opti.subject_to(X(:, 1) == X0);  % Initial condition

for k = 1:N
    % Unicycle dynamics
    x_next = X(:, k) + dt * [U(1, k) * cos(X(3, k));
                              U(1, k) * sin(X(3, k));
                              U(2, k)];
    opti.subject_to(X(:, k+1) == x_next);
end

% Control constraints
opti.subject_to(v_min <= U(1, :) <= v_max);
opti.subject_to(omega_min <= U(2, :) <= omega_max);

% CBF constraints
for k = 1:N
    for i = 1:n_obs_max
        % Barrier function: h = range - d_safe
        h = Ranges(i) - d_safe;

        % Time derivative: h_dot
        h_dot = -U(1, k) * cos(Bearings(i)) - Ranges(i) * U(2, k) * sin(Bearings(i));

        % CBF constraint: h_dot + alpha*h >= 0
        cbf_constraint = h_dot + alpha_cbf * h;

        if use_soft_cbf
            % Soft constraint: cbf_constraint + slack >= 0
            % Only active if i <= N_obs
            opti.subject_to(if_else(i <= N_obs, cbf_constraint + Slack(i, k), 1e6) >= 0);
        else
            % Hard constraint
            opti.subject_to(if_else(i <= N_obs, cbf_constraint, 1e6) >= 0);
        end
    end
end

% Solver options
opts = struct;
opts.ipopt.print_level = 0;
opts.ipopt.max_iter = 100;
opts.ipopt.tol = 1e-4;
opts.print_time = 0;
opts.ipopt.acceptable_tol = 1e-3;
opts.ipopt.warm_start_init_point = 'yes';

opti.solver('ipopt', opts);

fprintf('CasADi Opti problem built successfully!\n\n');

%% Simulation Loop
X_sim = x0';
U_sim = [];
x_current = x0;
lastMV = [0; 0];
last_X_traj = repmat(x0, 1, N+1);  % Store last state trajectory for warm start

% Store LiDAR data and solve times
scan_history = {};
solve_times = zeros(length(t)-1, 1);

fprintf('Starting NMPC simulation with CBF obstacle avoidance (CasADi Opti+IPOPT)...\n');
fprintf('Horizontal corridor: width %.1fm, length %.1fm\n', corridor_width, corridor_length);
fprintf('Obstacle at [%.1f, %.1f], size: %.2fm\n', obstacle_center(1), obstacle_center(2), obstacle_size);
fprintf('Safety radius: %.2fm\n', d_safe);

for k = 1:length(t)-1
    tic;

    %% Get LiDAR scan
    scan = lms_scan_new(x_current, obstacles, max_range, noise_std, lidar_model);
    scan_history{k} = scan;

    %% Prepare obstacle data
    % Filter scan: valid readings within threshold distance
    valid = ~isnan(scan(:,1)) & scan(:,1) < constraint_range;
    valid_idx = find(valid);
    valid_idx = valid_idx(1:scan_downsample:end);

    ranges_active = scan(valid_idx, 1);
    bearings_active = scan(valid_idx, 2);
    n_obstacles = length(ranges_active);

    % Pad to fixed size
    ranges_param = [ranges_active; 1e6*ones(n_obs_max - n_obstacles, 1)];
    bearings_param = [bearings_active; zeros(n_obs_max - n_obstacles, 1)];

    %% Reference over prediction horizon
    ref_idx = k:min(k+N, length(xref));
    xref_horizon = xref(ref_idx, :)';  % 3 x length

    % Pad if needed
    if size(xref_horizon, 2) < N+1
        n_pad = N+1 - size(xref_horizon, 2);
        xref_horizon = [xref_horizon, repmat(xref_horizon(:, end), 1, n_pad)];
    end

    %% Set parameter values
    opti.set_value(X0, x_current);
    opti.set_value(Xref, xref_horizon);
    opti.set_value(Ranges, ranges_param);
    opti.set_value(Bearings, bearings_param);
    opti.set_value(N_obs, n_obstacles);

    %% Set initial guess (warm start)
    % Controls: shift previous solution and repeat last
    U_guess = [repmat(lastMV, 1, N)];
    opti.set_initial(U, U_guess);

    % States: use previous trajectory shifted forward
    X_guess = [last_X_traj(:, 2:end), last_X_traj(:, end)];
    opti.set_initial(X, X_guess);

    % Slack variables: initialize to small value
    if use_soft_cbf
        opti.set_initial(Slack, 1e-6 * ones(n_obs_max, N));
    end

    %% Solve
    try
        sol = opti.solve();
        u_opt = sol.value(U);
        x_traj = sol.value(X);
        u = u_opt(:, 1);
        last_X_traj = x_traj;
    catch e
        warning('Solver failed at step %d: %s. Using last control.', k, e.message);
        u = lastMV;
    end

    solve_times(k) = toc;

    %% Apply control to vehicle (unicycle dynamics)
    x_current = x_current + dt * [u(1)*cos(x_current(3)); u(1)*sin(x_current(3)); u(2)];

    %% Store data
    X_sim = [X_sim; x_current'];
    U_sim = [U_sim; u'];
    lastMV = u;

    if mod(k, 20) == 0
        fprintf('Time: %.1fs, Pos: [%.2f, %.2f], Solve time: %.2f ms\n', ...
                t(k), x_current(1), x_current(2), solve_times(k)*1000);
    end
end

fprintf('\nSimulation complete!\n');
fprintf('Average solve time: %.2f ms\n', mean(solve_times)*1000);
fprintf('Max solve time: %.2f ms\n', max(solve_times)*1000);
fprintf('Min solve time: %.2f ms\n', min(solve_times)*1000);

%% Visualization
figure('Position', [100 100 1600 900]);
movegui('center');

%% 1. Corridor view with trajectory
subplot(2,3,1);
hold on; grid on; axis equal;

% Draw corridor walls (horizontal lines)
plot([-1, corridor_length], [corridor_width/2, corridor_width/2], 'k-', 'LineWidth', 2);
plot([-1, corridor_length], [-corridor_width/2, -corridor_width/2], 'k-', 'LineWidth', 2);

% Draw square obstacle
half_size = obstacle_size / 2;
x_square = obstacle_center(1) + [-half_size, half_size, half_size, -half_size, -half_size];
y_square = obstacle_center(2) + [-half_size, -half_size, half_size, half_size, -half_size];
plot(x_square, y_square, 'r-', 'LineWidth', 2);
fill(x_square, y_square, [1 0.8 0.8], 'FaceAlpha', 0.5, 'EdgeColor', 'r', 'LineWidth', 2);

% Plot trajectory
plot(xref(:,1), xref(:,2), 'g--', 'LineWidth', 2);
plot(X_sim(:,1), X_sim(:,2), 'b-', 'LineWidth', 1.5);
plot(X_sim(1,1), X_sim(1,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(X_sim(end,1), X_sim(end,2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

xlabel('$x$ [m]', 'Interpreter', 'latex');
ylabel('$y$ [m]', 'Interpreter', 'latex');
title('Horizontal Corridor Navigation (CasADi Opti)', 'Interpreter', 'latex');
legend('Wall', '', 'Obstacle', '', 'Reference', 'NMPC+CBF', 'Start', 'End', ...
       'Interpreter', 'latex', 'Location', 'best');
xlim([-1, 13]);
ylim([-3, 3]);

%% 2. LiDAR visualization at selected timesteps
subplot(2,3,2);
hold on; grid on; axis equal;

% Draw corridor and obstacle for reference
plot([-1, corridor_length], [corridor_width/2, corridor_width/2], 'k-', 'LineWidth', 1);
plot([-1, corridor_length], [-corridor_width/2, -corridor_width/2], 'k-', 'LineWidth', 1);
plot(x_square, y_square, 'r-', 'LineWidth', 1);

% Show LiDAR scans at a few timesteps
timesteps_to_show = [1, round(length(scan_history)/3), round(2*length(scan_history)/3), length(scan_history)];
colors = ['r', 'g', 'b', 'm'];

for i = 1:length(timesteps_to_show)
    k = timesteps_to_show(i);
    if k <= length(scan_history)
        scan = scan_history{k};
        x_robot = X_sim(k, :)';

        % Convert scan to Cartesian
        valid = ~isnan(scan(:,1));
        ranges = scan(valid, 1);
        bearings = scan(valid, 2);

        % Points in world frame
        x_points = x_robot(1) + ranges .* cos(bearings + x_robot(3));
        y_points = x_robot(2) + ranges .* sin(bearings + x_robot(3));

        plot(x_points, y_points, [colors(i), '.'], 'MarkerSize', 4);
        plot(x_robot(1), x_robot(2), [colors(i), 'o'], 'MarkerSize', 8, 'MarkerFaceColor', colors(i));
    end
end

xlabel('$x$ [m]', 'Interpreter', 'latex');
ylabel('$y$ [m]', 'Interpreter', 'latex');
title('LiDAR Scans at Different Times', 'Interpreter', 'latex');
xlim([-1, 13]);
ylim([-3, 3]);

%% 3. Control inputs
subplot(2,3,3);
plot(t(1:end-1), U_sim(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t(1:end-1), U_sim(:,2), 'r-', 'LineWidth', 1.5);
yline(v_max, 'b--', 'LineWidth', 1);
yline(omega_max, 'r--', 'LineWidth', 1);
yline(omega_min, 'r--', 'LineWidth', 1);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Control Input', 'Interpreter', 'latex');
title('Control Inputs', 'Interpreter', 'latex');
legend('$v$ [m/s]', '$\omega$ [rad/s]', 'Interpreter', 'latex');
grid on;

%% 4. Position tracking
subplot(2,3,4);
plot(t(1:end-1), X_sim(1:end-1,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t, xref(:,1), 'b--', 'LineWidth', 1.5);
plot(t(1:end-1), X_sim(1:end-1,2), 'r-', 'LineWidth', 1.5);
plot(t, xref(:,2), 'r--', 'LineWidth', 1.5);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Position [m]', 'Interpreter', 'latex');
title('Position Tracking', 'Interpreter', 'latex');
legend('$x$', '$x_{ref}$', '$y$', '$y_{ref}$', 'Interpreter', 'latex');
grid on;

%% 5. Distance to closest obstacle
subplot(2,3,5);
min_distances = zeros(length(scan_history), 1);
for k = 1:length(scan_history)
    scan = scan_history{k};
    valid = ~isnan(scan(:,1));
    if any(valid)
        min_distances(k) = min(scan(valid, 1));
    else
        min_distances(k) = NaN;
    end
end
plot(t(1:end-1), min_distances, 'k-', 'LineWidth', 1.5); hold on;
yline(d_safe, 'r--', 'LineWidth', 2, 'DisplayName', 'Safety radius');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Distance [m]', 'Interpreter', 'latex');
title('Minimum Distance to Obstacles', 'Interpreter', 'latex');
legend('Min distance', 'Safety threshold', 'Interpreter', 'latex');
grid on;

%% 6. Solve time over simulation
subplot(2,3,6);
plot(t(1:end-1), solve_times*1000, 'k-', 'LineWidth', 1.5); hold on;
yline(dt*1000, 'r--', 'LineWidth', 2, 'DisplayName', 'Real-time limit');
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Solve Time [ms]', 'Interpreter', 'latex');
title('Optimization Solve Time', 'Interpreter', 'latex');
legend('Solve time', sprintf('dt = %.0f ms', dt*1000), 'Interpreter', 'latex');
grid on;

sgtitle('NMPC with CBF (CasADi Opti+IPOPT)', 'Interpreter', 'latex');

%% Animation
figure('Position', [100 100 1200 600]);
movegui('center');
hold on; grid on; axis equal;

% Draw static elements
plot([-1, corridor_length], [corridor_width/2, corridor_width/2], 'k-', 'LineWidth', 3);
plot([-1, corridor_length], [-corridor_width/2, -corridor_width/2], 'k-', 'LineWidth', 3);
plot(x_square, y_square, 'r-', 'LineWidth', 3);
fill(x_square, y_square, [1 0.8 0.8], 'FaceAlpha', 0.5, 'EdgeColor', 'r', 'LineWidth', 2);

% Reference trajectory
plot(xref(:,1), xref(:,2), 'g--', 'LineWidth', 1.5, 'DisplayName', 'Reference');

% Actual trajectory (will be drawn progressively)
traj_line = plot(NaN, NaN, 'b-', 'LineWidth', 2, 'DisplayName', 'Actual Trajectory');

% Robot visualization
robot_size = 0.3;
robot_body = plot(NaN, NaN, 'bo', 'MarkerSize', 15, 'MarkerFaceColor', 'b', 'DisplayName', 'Robot');
robot_heading = quiver(NaN, NaN, NaN, NaN, 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 0.8);

% LiDAR rays (show subset for clarity)
lidar_rays = cell(1, 18);  % Show every 10th ray
for i = 1:18
    lidar_rays{i} = plot(NaN, NaN, 'c-', 'LineWidth', 0.5, 'Color', [0 0.8 0.8 0.3]);
end
lidar_points = plot(NaN, NaN, 'r.', 'MarkerSize', 4);

% Safety circle
safety_circle = plot(NaN, NaN, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Safety Radius');

xlabel('$x$ [m]', 'Interpreter', 'latex', 'FontSize', 14);
ylabel('$y$ [m]', 'Interpreter', 'latex', 'FontSize', 14);
title('LiDAR CBF + NMPC (CasADi Opti+IPOPT)', 'Interpreter', 'latex', 'FontSize', 16);
legend('Walls', '', 'Obstacle', '', 'Reference', 'Location', 'northeast');
xlim([-1, 13]);
ylim([-3, 3]);

% Time display
time_text = text(0.5, 2.5, '', 'FontSize', 12, 'BackgroundColor', 'white');

% Animation loop
playback_speed = 3;  % Play every Nth frame for speed
fprintf('\nPlaying animation...\n');

for k = 1:playback_speed:length(X_sim(:,1))
    % Update trajectory
    set(traj_line, 'XData', X_sim(1:k, 1), 'YData', X_sim(1:k, 2));

    % Robot position and heading
    x_rob = X_sim(k, 1);
    y_rob = X_sim(k, 2);
    theta_rob = X_sim(k, 3);

    set(robot_body, 'XData', x_rob, 'YData', y_rob);
    set(robot_heading, 'XData', x_rob, 'YData', y_rob, ...
        'UData', 0.4*cos(theta_rob), 'VData', 0.4*sin(theta_rob));

    % Safety circle
    theta_circle = linspace(0, 2*pi, 50);
    safety_x = x_rob + d_safe * cos(theta_circle);
    safety_y = y_rob + d_safe * sin(theta_circle);
    set(safety_circle, 'XData', safety_x, 'YData', safety_y);

    % LiDAR visualization
    if k <= length(scan_history)
        scan = scan_history{k};
        valid = ~isnan(scan(:,1));

        if any(valid)
            ranges = scan(valid, 1);
            bearings = scan(valid, 2);

            x_points = x_rob + ranges .* cos(bearings + theta_rob);
            y_points = y_rob + ranges .* sin(bearings + theta_rob);

            set(lidar_points, 'XData', x_points, 'YData', y_points);

            % Show subset of rays
            ray_indices = 1:10:length(ranges);
            for i = 1:min(length(ray_indices), 18)
                idx = ray_indices(i);
                if idx <= length(ranges)
                    set(lidar_rays{i}, 'XData', [x_rob, x_points(idx)], ...
                        'YData', [y_rob, y_points(idx)]);
                end
            end
        end
    end

    % Update time
    if k <= length(t)
        set(time_text, 'String', sprintf('Time: %.1f s', t(k)));
    end

    drawnow;
    pause(0.02);
end

fprintf('Animation complete!\n');
