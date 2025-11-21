function [delta_d, delta_beta] = apolo_odometry(robot, prev_odometry)
%APOLO_ODOMETRY - Computes odometry increments between time steps
%   Calculates the displacement [Δd, Δβ] in odometry readings from the
%   Apolo simulator between two consecutive time steps.
%
%   This function should be called AFTER:
%     1. Recording previous odometry with apoloGetOdometry()
%     2. Sending velocity command with apoloMoveMRobot()
%     3. Updating simulation with apoloUpdate()
%
% Input Arguments:
%   robot (char)          - Name of the robot in the Apolo simulator
%   prev_odometry (1x3)   - Previous odometry reading [x, y, theta] in [m, m, rad]
%
% Output Arguments:
%   delta_d (double)      - Distance increment Δd [m] (Euclidean norm)
%   delta_beta (double)   - Heading increment Δβ [rad]
%
% Example:
%   % Record initial odometry
%   prev_odom = apoloGetOdometry('robot1');
%
%   % Send velocity command and update simulation
%   apoloMoveMRobot('robot1', [0.5, 0.1], dt);
%   apoloUpdate();
%
%   % Compute odometry difference
%   [delta_d, delta_beta] = calculateOdometryDiff('robot1', prev_odom);
%
% See also: apoloGetOdometry, apoloMoveMRobot, apoloUpdate

arguments
    robot {mustBeTextScalar}
    prev_odometry (1,3) double {mustBeReal, mustBeFinite}
end

%% Get Current Odometry
current_odometry = apoloGetOdometry(robot);

%% Compute Odometry Increments

% Position change in world frame
delta_x = current_odometry(1) - prev_odometry(1);
delta_y = current_odometry(2) - prev_odometry(2);

% Distance increment (Euclidean norm of position change)
delta_d = norm([delta_x, delta_y]);

% Heading increment
delta_beta = current_odometry(3) - prev_odometry(3);

end
