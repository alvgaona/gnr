%% Extended Kalman Filter with Differential Drive Model
% Control input: [Δd, Δβ] where Δd is distance traveled and Δβ is heading change
% This is typical for odometry-based systems

clear;
close all;
clc;

%% Vehicle and Simulation Parameters
WorldXML = readstruct("DafneEKFLong.xml","FileType","xml");
time_step = 0.2;            % Discrete time step [s]
simulation_time = 30;       % Total simulation time [s]
num_steps = simulation_time / time_step;
robotName = convertStringsToChars(WorldXML.World.Pioneer3ATSim.nameAttribute);%LMS100Sim %LandMark mark_id="1"
laserName = convertStringsToChars(WorldXML.World.LMS100Sim.nameAttribute);%'LMS100';
%% Reset Odom
apoloResetOdometry(robotName);

%% Define Trajectory
% You can change these to create different trajectories
trajectory_type = 'linear';  % Options: 'linear', 'circular', 'curve'

switch trajectory_type
    case 'linear'
        % Linear trajectory
        velocity_profile = 1.5 * ones(1, num_steps);        % Constant 2 m/s
        angular_velocity_profile = 0.0 * ones(1, num_steps); % Zero rotation
    case 'circular'
        % Circular/Arc trajectory
        velocity_profile = 1.0 * ones(1, num_steps); % Constant 1 m/s
        radius = 3.2; % 5m radius
        angular_velocity_profile = (velocity_profile / radius); % Constant angular velocity
    case 'curve'
        % Curved path with varying angular velocity
        velocity_profile = 1.5 * ones(1, num_steps); % Constant 1.5 m/s
        angular_velocity_profile = 0.2 * sin(2*pi*(0:num_steps-1)/num_steps); % Sinusoidal rotation [rad/s]
end


%% Process Noise (Motion Model Uncertainty)

% For EKF (in odometry space: Δd, Δβ) Obtained from calculateDiffOdometry
process_noise_d = 9.5003e-05;      % Distance increment noise [m]
process_noise_beta = 3.9080e-05;   % Heading increment noise [rad]
Q = [process_noise_d^2, 0;
     0, process_noise_beta^2];

% Pre-compute standard deviations for efficiency
Q_std = sqrt(diag(Q));

%% Measurement Noise (Sensor Uncertainty)

measurement_noise_std = 0.023174091647608;  % Bearing measurement noise [rad]
R = measurement_noise_std^2 * eye(3);  % 3 beacons

% Pre-compute standard deviations for efficiency
R_std = sqrt(diag(R));

%% Beacon Positions
num_beacons = size(WorldXML.World.LandMark,2);
if num_beacons == 0
    beacons = [ %x y id
        -3.9  17 1;   % Beacon 1
        3.9   17 2;   % Beacon 2
        0     17 3   % Beacon 3
    ];
    num_beacons=size(beacons, 1);
else %Get beacon positions from file
    beacons = zeros(num_beacons,3);
    for i=1:num_beacons
        coords = split(extractBetween(WorldXML.World.LandMark(i).position,"{","}"),",");
        beacons(i,:) = [coords(1) coords(2) WorldXML.World.LandMark(i).mark_idAttribute];
    end
end
beacons = sort(beacons,3); %make index in array coincide with landmark index

%% Initial Conditions

% Initial state
if apoloPlaceMRobot(robotName,[0,-2.4,0],pi/2)~=1
    disp("Error placing "+robotName+" on position");
    return
end
apoloLoc = apoloGetLocationMRobot(robotName);%[x y z theta]
true_state = [apoloLoc(1);apoloLoc(2);apoloLoc(4)];%[0; 0; pi/4];  % [x, y, theta]
apoloResetOdometry(robotName,true_state');
apoloUpdate();

% Initial state prediction (with error for EKF)
estimated_state = true_state + [0.5; 0.5; 0.1];  % Just deviate a little

% Initial state covariance: with uncertainty for the state vector
P = diag([0.5^2, 0.5^2, 0.1^2]);

%% Variables to show results
true_trajectory = zeros(3, num_steps);
estimated_trajectory = zeros(3, num_steps);
variance_history = zeros(3, num_steps);
control_history = zeros(2, num_steps);

%% Main Simulation Loop

for step = 1:num_steps
    % 1. Simulate ground truth robot motion: Differential drive model with
    % linear and angular velocity as control input, [v, ω]

    % Differential drive control inputs: linear velocity (v) and angular velocity (ω)
    v = velocity_profile(step);
    omega = angular_velocity_profile(step);

    % Differential drive dynamics (no noise - perfect execution)
    % Note: v and ω could come from wheel velocities: v = (v_R + v_L)/2, ω = (v_R - v_L)/b

    % Use Euler method for integration (simpler, first-order)
    theta_k = true_state(3);

    % meas=apoloGetOdometry(robotName);
    % apoloMoveMRobot(robotName,[linearVelCmd,angularVelCmd],Atime);
    % apoloUpdate();
    % [d,B]=calculateOdometryDiff(robotName,meas);
    % Update robot with specified commands
    prev_odom= apoloGetOdometry(robotName);
    apoloMoveMRobot(robotName,[v omega],time_step);
    apoloUpdate();
    pause(time_step);
    apoloLoc = apoloGetLocationMRobot(robotName);%[x y z theta]
    true_state = [apoloLoc(1);apoloLoc(2);apoloLoc(4)];%[0; 0; pi/4];  % [x, y, theta]

    %% Compute Odometry Measurements [Δd, Δβ] from true motion - Vectorized
    % This is what the EKF actually observes (with noise)
    % state_delta = true_state - prev_state;
    % actual_control = [norm(state_delta(1:2)); state_delta(3)];

    true_trajectory(:, step) = true_state;

    %% Generate Measurements (Bearing to each beacon) - Vectorized
    % Compute all bearings at once
    measurements = apoloGetLaserLandMarks(laserName).angle';
    measurementIDs = apoloGetLaserLandMarks(laserName).id;
    % Normalize to [-pi, pi]
    measurements = atan2(sin(measurements), cos(measurements));

    %% EKF PREDICTION STEP
    % Control input: u = [Δd, Δβ]
    [delta_d_hat,delta_beta_hat]=calculateOdometryDiff(robotName,prev_odom);
    theta_k = estimated_state(3);

    % Predicted state using midpoint odometry model
    theta_mid = theta_k + delta_beta_hat / 2;
    predicted_state = [
        estimated_state(1) + delta_d_hat * cos(theta_mid);
        estimated_state(2) + delta_d_hat * sin(theta_mid);
        theta_k + delta_beta_hat
    ];

    odometryState = apoloGetOdometry(robotName);

    % Compute Jacobian of discrete transition function with respect to state
    % f(x,y,θ) = [x + Δd*cos(θ + Δβ/2); y + Δd*sin(θ + Δβ/2); θ + Δβ]
    A = [
        1, 0, -delta_d_hat * sin(theta_mid);
        0, 1,  delta_d_hat * cos(theta_mid);
        0, 0,  1
    ];

    % Compute Jacobian with respect to control input u = [Δd, Δβ]
    % For midpoint model: ∂f/∂[Δd, Δβ]
    W = [
        cos(theta_mid),  -0.5 * delta_d_hat * sin(theta_mid);
        sin(theta_mid),   0.5 * delta_d_hat * cos(theta_mid);
        0,                1
    ];

    % Predict covariance
    P_predicted = A * P * A' + W * Q * W';

    %% EKF UPDATE STEP - Vectorized
    % Compute all predicted measurements at once
    dx_pred = zeros(size(measurementIDs,2),1);
    dy_pred = dx_pred;
    for i=1:size(measurementIDs,2)
        ji = measurementIDs(i);
        dx_pred(i) = beacons(ji,1) - predicted_state(1);%get predicted only for measured beacons
        dy_pred(i) = beacons(ji,2) - predicted_state(2);
    end
    dist_sq = dx_pred.^2 + dy_pred.^2;

    % Predicted bearings
    predicted_bearings = atan2(dy_pred, dx_pred);
    predicted_measurements = predicted_bearings - predicted_state(3);

    % Normalize to [-pi, pi]
    predicted_measurements = atan2(sin(predicted_measurements), cos(predicted_measurements));

    % Measurement Jacobian (all rows at once)
    H = [dy_pred ./ dist_sq, -dx_pred ./ dist_sq, -ones(size(measurementIDs,2), 1)];

    % Innovation (measurement residual) with angle normalization
    innovation = measurements - predicted_measurements;
    innovation = atan2(sin(innovation), cos(innovation));

    % Innovation covariance
    R = measurement_noise_std^2 * eye(size(measurementIDs,2));  % calculate R based on n beacons
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
figure('Name', 'EKF with Ackermann Model (Control: Δd, Δβ)', 'Position', [50 50 1400 900]);

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
figure('Name', 'Control Inputs', 'Position', [100 100 1000 800]);

% True Differential Drive control inputs [v, ω]
subplot(2,2,1);
plot(velocity_profile, 'r-', 'LineWidth', 1.5);
xlabel('Time Step'); ylabel('Velocity [m/s]');
title('True Differential Drive: Linear Velocity Command');
grid on;

subplot(2,2,2);
plot(rad2deg(angular_velocity_profile), 'r-', 'LineWidth', 1.5);
xlabel('Time Step'); ylabel('Angular Velocity [deg/s]');
title('True Differential Drive: Angular Velocity Command');
grid on;

% Odometry measurements (what EKF sees) [Δd, Δβ]
subplot(2,2,3);
plot(control_history(1,:), 'b-', 'LineWidth', 1.5);
xlabel('Time Step'); ylabel('Δd [m]');
title('EKF Input: Measured Distance Increment');
legend('Odometry (noisy)'); grid on;

subplot(2,2,4);
plot(rad2deg(control_history(2,:)), 'b-', 'LineWidth', 1.5);
xlabel('Time Step'); ylabel('Δβ [deg]');
title('EKF Input: Measured Heading Increment');
legend('Odometry (noisy)'); grid on;

%% Display Statistics
fprintf('\n========== EKF with Differential Drive Model - Results ==========\n');
fprintf('True Dynamics:          Differential Drive [v, ω]\n');
fprintf('EKF Control Input:      Odometry [Δd, Δβ]\n');
fprintf('Trajectory Type:        %s\n', trajectory_type);
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
