function [v, omega] = differential_drive_pid(gains, desired_pose, current_pose, dt)

arguments
    gains (1,1) struct {mustHavePIDFields}
    desired_pose (1,3) double {mustBeReal, mustBeFinite}
    current_pose (1,3) double {mustBeReal, mustBeFinite}
    dt (1,1) double {mustBePositive, mustBeFinite}
end

%DIFFERENTIAL_DRIVE_PID - Pose-to-velocity controller for differential drive robots
%   Computes velocity commands to drive the robot toward a desired pose
%   (position + heading) using dual independent PID controllers.
%
%   This controller simultaneously:
%     1. Reduces Euclidean distance to target position (x_d, y_d)
%     2. Aligns robot heading to desired orientation theta_d
%
%   The robot will follow a curved trajectory while both translating toward
%   the goal position and rotating toward the desired heading. This is NOT
%   a pure pursuit controller (it does not rotate toward the target first).
%
% Input Arguments:
%   gains (struct)       - PID gains for both control loops
%                          Required fields:
%                            Kp_v, Ki_v, Kd_v      - Linear velocity PID gains
%                            Kp_omega, Ki_omega, Kd_omega - Angular velocity PID gains
%   desired_pose (1x3)   - Target pose [x_d, y_d, theta_d] in world frame [m, m, rad]
%   current_pose (1x3)   - Current robot pose [x, y, theta] in world frame [m, m, rad]
%   dt (double)          - Time step for derivative/integral computation [s]
%
% Output Arguments:
%   v (double)           - Linear velocity command [m/s], range: [0, v_max]
%   omega (double)       - Angular velocity command [rad/s], range: [-omega_max, omega_max]
%
% Example:
%   gains.Kp_v = 0.5; gains.Ki_v = 0.01; gains.Kd_v = 0.1;
%   gains.Kp_omega = 2.0; gains.Ki_omega = 0.05; gains.Kd_omega = 0.3;
%   [v, omega] = differential_drive_pid(gains, [5, 3, pi/4], [0, 0, 0], 0.1);

%% PID Controller Parameters

% Maximum control inputs
v_max = 2.0;        % Maximum linear velocity (m/s)
omega_max = 2.0;    % Maximum angular velocity (rad/s)

%% Error Terms Initialization
% Persistent variables maintain state between function calls (similar to
% 'static' in C), avoiding the need to pass and return these variables

persistent error_dist_prev error_dist_integral;
persistent error_heading_prev error_heading_integral;

% Initialize on first call
if isempty(error_dist_integral)
    error_dist_prev = 0;
    error_dist_integral = 0;
    error_heading_prev = 0;
    error_heading_integral = 0;
end

%% Compute Error Terms

% Position errors
error_x = desired_pose(1) - current_pose(1);
error_y = desired_pose(2) - current_pose(2);

% Distance error (Euclidean norm)
error_dist = sqrt(error_x^2 + error_y^2);

% Heading error (desired - actual)
error_heading = desired_pose(3) - current_pose(3);

% Normalize angle error to [-pi, pi]
error_heading = atan2(sin(error_heading), cos(error_heading));

%% PID for Linear Velocity (Distance Error)

% Compute PID terms
error_dist_derivative = (error_dist - error_dist_prev) / dt;
error_dist_integral = error_dist_integral + error_dist * dt;

% Linear velocity command
v = gains.Kp_v * error_dist + ...
    gains.Ki_v * error_dist_integral + ...
    gains.Kd_v * error_dist_derivative;

%% PID for Angular Velocity (Heading Error)

% Compute PID terms
error_heading_derivative = (error_heading - error_heading_prev) / dt;
error_heading_integral = error_heading_integral + error_heading * dt;

% Angular velocity command
omega = gains.Kp_omega * error_heading + ...
        gains.Ki_omega * error_heading_integral + ...
        gains.Kd_omega * error_heading_derivative;

%% Update State and Saturate Outputs

% Store errors for next iteration
error_dist_prev = error_dist;
error_heading_prev = error_heading;

% Saturate control inputs to physical limits
v = max(min(v, v_max), 0);                       % Linear velocity: [0, v_max]
omega = max(min(omega, omega_max), -omega_max);  % Angular velocity: [-omega_max, omega_max]
end

%% Validation Functions

function mustHavePIDFields(gains)
    % Validate that gains struct has all required PID fields
    required_fields = {'Kp_v', 'Ki_v', 'Kd_v', 'Kp_omega', 'Ki_omega', 'Kd_omega'};

    for i = 1:numel(required_fields)
        if ~isfield(gains, required_fields{i})
            error('differential_drive_pid:MissingField', ...
                  'gains struct must have field "%s"', required_fields{i});
        end

        % Validate that each field is a scalar numeric value
        field_value = gains.(required_fields{i});
        if ~isnumeric(field_value) || ~isscalar(field_value) || ~isfinite(field_value)
            error('differential_drive_pid:InvalidFieldValue', ...
                  'gains.%s must be a finite scalar numeric value', required_fields{i});
        end
    end
end
