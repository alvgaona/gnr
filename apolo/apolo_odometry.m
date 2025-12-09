function [delta_d, delta_beta] = apolo_odometry(robot, prev_odometry)
%APOLO_ODOMETRY - Computes odometry increments between time steps
%   Calculates the displacement [Δd, Δβ] in odometry readings from the
%   Apolo simulator between two consecutive time steps.
%
% Input Arguments:
%   robot (char)          - Name of the robot in the Apolo simulator
%   prev_odometry (1x3)   - Previous odometry reading [x, y, theta] in [m, m, rad]
%
% Output Arguments:
%   delta_d (double)      - Distance increment Δd [m] (arc length traveled)
%   delta_beta (double)   - Heading increment Δβ [rad]

arguments
    robot {mustBeTextScalar}
    prev_odometry (1,3) double {mustBeReal, mustBeFinite}
end

%% Get Current Odometry
current_odometry = apoloGetOdometry(robot);

%% Compute Odometry Increments

% Heading increment
delta_beta = current_odometry(3) - prev_odometry(3);

% Normalize to [-pi, pi]
delta_beta = atan2(sin(delta_beta), cos(delta_beta));

% Position change in world frame
delta_x = current_odometry(1) - prev_odometry(1);
delta_y = current_odometry(2) - prev_odometry(2);

% Distance increment:
% 1. Use arc length for turns
% 2. Euclidean for straight motion
ANGLE_THRESHOLD = 1e-4;  % rad (~0.006 degrees)

if abs(delta_beta) > ANGLE_THRESHOLD
    chord_length = norm([delta_x, delta_y]);
    delta_d = chord_length * abs(delta_beta) / (2 * sin(abs(delta_beta)/2));
else
    % Nearly straight motion: arc length ≈ Euclidean distance
    delta_d = norm([delta_x, delta_y]);
end

% Preserve sign of motion (forward/backward)
theta_mid = prev_odometry(3) + delta_beta / 2;
displacement_forward = delta_x * cos(theta_mid) + delta_y * sin(theta_mid);
if displacement_forward < 0
    delta_d = -delta_d;
end

end
