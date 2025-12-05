clc; clear; close all;

%% Environment. Horizontal corridor with parallel walls
corridor_width = 4;  % Corridor width [m]
corridor_length = 12; % Corridor length [m]

% Define obstacles as cell array
obstacles = {};

% Corridor walls (finite line segments)
obstacles{end+1} = struct('type', 'wall', 'p1', [-1; corridor_width/2], 'p2', [corridor_length; corridor_width/2]);
obstacles{end+1} = struct('type', 'wall', 'p1', [-1; -corridor_width/2], 'p2', [corridor_length; -corridor_width/2]);

% Square obstacle in center of corridor
obstacle_center = [5; 0]; % Center position [x; y] (centered in corridor)
obstacle_size = 1.0;        % Square side length [m] (reduced from 1.2)
obstacles{end+1} = struct('type', 'square', 'center', obstacle_center, 'size', obstacle_size);

%% LiDAR Configuration
max_range = 10.0;       % Maximum sensor range [m]
noise_std = 0.02;       % Range measurement noise [m]
lidar_model = 'LMS100'; % 180-degree FOV, 181 beams

%% NMPC Configuration
controller = NMPCCBFController(...
    'HorizonLength', 20, ...
    'TimeStep', 0.1, ...
    'StateWeights', [1, 1, 0.1], ...
    'ControlWeights', [1, 1], ...
    'VelocityLimits', [0, 1], ...
    'AngularLimits', [-2, 2], ...
    'SafetyRadius', 0.6, ...
    'AlphaCBF', 0.5, ...
    'ScanDownsample', 100, ...
    'ConstraintRange', 3.0, ...
    'MaxIterations', 100);

% Extract parameters for simulation and plotting
dt = controller.dt;
d_safe = controller.d_safe;
v_max = controller.v_max;
omega_min = controller.omega_min;
omega_max = controller.omega_max;

%% Reference Trajectory
Tsim = 100;
t = 0:dt:Tsim;

% Straight horizontal trajectory through center of corridor
v_ref = 0.8;                 % Reference speed [m/s]
x_ref = v_ref * t;           % Move horizontally at constant speed
y_ref = zeros(size(t));      % Stay centered in corridor
theta_ref = zeros(size(t));  % Point along x-axis (0 degrees)

xref = [x_ref', y_ref', theta_ref'];

%% Initial State
x0 = [0; 0; 0];  % Start at left of corridor, centered, pointing right

%% Simulation Loop
X = x0';
U = [];
x = x0;

% Store LiDAR data for visualization
scan_history = {};

fprintf('Starting NMPC simulation with CBF obstacle avoidance...\n');
fprintf('Horizontal corridor: width %.1fm, length %.1fm\n', corridor_width, corridor_length);
fprintf('Obstacle at [%.1f, %.1f], size: %.2fm\n', obstacle_center(1), obstacle_center(2), obstacle_size);
fprintf('Safety radius: %.2fm\n', d_safe);

for k = 1:length(t)-1
    %% Get LiDAR scan
    scan = lms_scan_new(x, obstacles, max_range, noise_std, lidar_model);
    scan_history{k} = scan;

    % Compute control using:
    % x: current state
    % xref: from current iteration onwards
    % scan: measurements scanned in current iteration
    u = controller.compute(x, xref(k, :), scan);

    %% Apply control to vehicle (unicycle dynamics)
    x = x + dt * [u(1)*cos(x(3)); u(1)*sin(x(3)); u(2)];

    %% Store data
    X = [X; x'];
    U = [U; u'];

    if mod(k, 20) == 0
        fprintf('Time: %.1fs, Pos: [%.2f, %.2f]\n', t(k), x(1), x(2));
    end
end

fprintf('Simulation complete!\n');

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
plot(X(:,1), X(:,2), 'b-', 'LineWidth', 1.5);
plot(X(1,1), X(1,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
plot(X(end,1), X(end,2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');

xlabel('$x$ [m]', 'Interpreter', 'latex');
ylabel('$y$ [m]', 'Interpreter', 'latex');
title('Horizontal Corridor Navigation', 'Interpreter', 'latex');
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
        x_robot = X(k, :)';

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
plot(t(1:end-1), U(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t(1:end-1), U(:,2), 'r-', 'LineWidth', 1.5);
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
plot(t(1:end-1), X(1:end-1,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t, xref(:,1), 'b--', 'LineWidth', 1.5);
plot(t(1:end-1), X(1:end-1,2), 'r-', 'LineWidth', 1.5);
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

%% 6. Trajectory errors
subplot(2,3,6);
error_x = xref(1:end-1,1) - X(1:end-1,1);
error_y = xref(1:end-1,2) - X(1:end-1,2);
error_pos = sqrt(error_x.^2 + error_y.^2);

plot(t(1:end-1), error_pos, 'k-', 'LineWidth', 1.5);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Error [m]', 'Interpreter', 'latex');
title('Position Tracking Error', 'Interpreter', 'latex');
grid on;

sgtitle('NMPC with CBF Obstacle Avoidance in Corridor', 'Interpreter', 'latex');

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
title('LiDAR CBF + NMPC Path Following', 'Interpreter', 'latex', 'FontSize', 16);
legend('Walls', '', 'Obstacle', '', 'Reference', 'Location', 'northeast');
xlim([-1, 13]);
ylim([-3, 3]);

% Time display
time_text = text(0.5, 2.5, '', 'FontSize', 12, 'BackgroundColor', 'white');

% Animation loop
playback_speed = 3;  % Play every Nth frame for speed
fprintf('\nPlaying animation...\n');

for k = 1:playback_speed:length(X(:,1))
    % Update trajectory
    set(traj_line, 'XData', X(1:k, 1), 'YData', X(1:k, 2));

    % Robot position and heading
    x_rob = X(k, 1);
    y_rob = X(k, 2);
    theta_rob = X(k, 3);

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
