clc; clear; close all;

controller = NMPCController( ...
    PredictionHorizon=10, ...
    ControlHorizon=5, ...
    TimeStep=0.1, ...
    StateWeights=[10, 10, 1], ...
    ControlWeights=[0.1, 1], ...
    VelocityLimits=[0, 2], ...
    AngularLimits=[-2, 2] ...
);

x0 = [-1; 1.5; 0];      % Start a little bit away from the trajectory
Tsim = 15;              % The total time of the simulation
t = 0:controller.dt:Tsim; % Array of timesteps

% Reference trajectory
v_ref = 1.0;                   % Reference speed [m/s]
x_ref = v_ref * t;             % Move forward at constant speed
y_ref = 2 * sin(0.4 * x_ref);  % Smooth sinusoidal lateral motion

% Heading from tangent
dx = v_ref;
dy = 2 * 0.4 * cos(0.4 * x_ref);  % dy/dt = dy/dx * dx/dt
theta_ref = atan2(dy, dx);

xref = [x_ref', y_ref', theta_ref'];

%% Simulation
X = x0';
U = [];
x = x0;

for k = 1:length(t)-1
    u = controller.compute(x, xref(k:end,:));

    % Apply control to the vehicle
    x = x + controller.dt * [u(1)*cos(x(3)); u(1)*sin(x(3)); u(2)];

    X = [X; x'];
    U = [U; u'];
end

% Tracking errors
t_sim = t(1:end-1);
xref_sim = xref(1:end-1,:);
error_x = xref_sim(:,1) - X(1:end-1,1);
error_y = xref_sim(:,2) - X(1:end-1,2);
error_theta = xref_sim(:,3) - X(1:end-1,3);
error_theta = atan2(sin(error_theta), cos(error_theta));  % Normalize to [-pi, pi]

%% 9. Plotting
figure('Position', [100 100 1400 900]);

% Trajectory plot
subplot(3,3,1);
plot(xref(:,1), xref(:,2), 'k--', 'LineWidth', 2); hold on;
plot(X(:,1), X(:,2), 'b-', 'LineWidth', 1.5);
plot(X(1,1), X(1,2), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');
xlabel('$x$ [m]', 'Interpreter', 'latex');
ylabel('$y$ [m]', 'Interpreter', 'latex');
title('Trajectory', 'Interpreter', 'latex');
legend('Reference', 'NMPC', 'Start', 'Interpreter', 'latex');
grid on; axis equal;

% Linear velocity
subplot(3,3,2);
plot(t_sim, U(:,1), 'b-', 'LineWidth', 1.5);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('$v$ [m/s]', 'Interpreter', 'latex');
title('Linear Velocity', 'Interpreter', 'latex');
grid on;

% Angular velocity
subplot(3,3,3);
plot(t_sim, U(:,2), 'r-', 'LineWidth', 1.5);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('$\omega$ [rad/s]', 'Interpreter', 'latex');
title('Angular Velocity', 'Interpreter', 'latex');
grid on;

% X position error
subplot(3,3,4);
plot(t_sim, error_x, 'b-', 'LineWidth', 1.5);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('$e_x$ [m]', 'Interpreter', 'latex');
title('Position Error', 'Interpreter', 'latex');
grid on;

% Y position error
subplot(3,3,5);
plot(t_sim, error_y, 'r-', 'LineWidth', 1.5);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('$e_y$ [m]', 'Interpreter', 'latex');
title('Position Error', 'Interpreter', 'latex');
grid on;

% Heading error
subplot(3,3,6);
plot(t_sim, rad2deg(error_theta), 'm-', 'LineWidth', 1.5);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('$e_\theta$ [deg]', 'Interpreter', 'latex');
title('Heading Error', 'Interpreter', 'latex');
grid on;

% Total position error
subplot(3,3,7);
error_pos = sqrt(error_x.^2 + error_y.^2);
plot(t_sim, error_pos, 'k-', 'LineWidth', 1.5);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Error [m]', 'Interpreter', 'latex');
title('Total Position Error', 'Interpreter', 'latex');
grid on;

% Control inputs together
subplot(3,3,8);
plot(t_sim, U(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t_sim, U(:,2), 'r-', 'LineWidth', 1.5);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Control Input', 'Interpreter', 'latex');
title('Control Inputs', 'Interpreter', 'latex');
legend('$v$ [m/s]', '$\omega$ [rad/s]', 'Interpreter', 'latex');
grid on;

% X and Y positions
subplot(3,3,9);
plot(t, xref(:,1), 'b--', 'LineWidth', 1.5); hold on;
plot(t(1:end-1), X(1:end-1,1), 'b-', 'LineWidth', 1.5);
plot(t, xref(:,2), 'r--', 'LineWidth', 1.5);
plot(t(1:end-1), X(1:end-1,2), 'r-', 'LineWidth', 1.5);
xlabel('Time [s]', 'Interpreter', 'latex');
ylabel('Position [m]', 'Interpreter', 'latex');
title('$x$ and $y$ Positions', 'Interpreter', 'latex');
legend('$x_{\mathrm{ref}}$', '$x$', '$y_{\mathrm{ref}}$', '$y$', 'Interpreter', 'latex');
grid on;

sgtitle('NMPC Trajectory Tracking with Unicycle Dynamics', 'Interpreter', 'latex');
