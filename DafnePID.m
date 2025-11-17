function [v,omega] = DafnePID(gainStruct, desiredCoords,actualCoords,dt)
%PID - Calculates linear and angular velocity commands via PID
%   Calculates the linear and angular velocity commands based on the input
%and pose estimation
%
% Input Arguments:
%  - gainStruct (struct) -
%    Gains for the v and omega controllers (Kp, Ki and Kd for each)
%  - desiredCoords (double[]) -
%    Desired position for the robot at the moment [x y theta]
%  - actualCoords (double[]) -
%    Estimated position of the robot [x y theta]
%  - dt (double)
%    Time step (in seconds)
% 
% Output Arguments:
%  - v (double) - linear velocity command
%  - omega (double) - angular velocity command
%% PID Controller Parameters

% Maximum control inputs
v_max = 2.0;        % Maximum linear velocity (m/s)
omega_max = 2.0;    % Maximum angular velocity (rad/s)

% Error terms initialization ('persistent' is functionally equivalent to
% 'static' in C. Doing this prevents us from having to get and pass this 4
% extra arguments)
persistent error_dist_prev;
persistent error_dist_integral;
persistent error_heading_prev;
persistent error_heading_integral;

if isempty(error_dist_integral)
    error_dist_prev = 0;
    error_dist_integral = 0;
    error_heading_prev = 0;
    error_heading_integral = 0;
end

%% Controller logic
error_x = desiredCoords(1) - actualCoords(1);
error_y = desiredCoords(2) - actualCoords(2);

% Distance error (magnitude)
error_dist = sqrt(error_x^2 + error_y^2);

% Heading error to target
%theta_target = atan2(error_y, error_x);
% Normalize theta to [-pi, pi]
%theta = atan2(sin(actualCoords(3)), cos(actualCoords(3)));
%desTheta = atan2(sin(desiredCoords(3)), cos(desiredCoords(3)));
error_heading = desiredCoords(3) - actualCoords(3);

% Normalize angle error to [-pi, pi]
error_heading = atan2(sin(error_heading), cos(error_heading));

% PID for linear velocity (based on distance error)
error_dist_derivative = (error_dist - error_dist_prev) / dt;
error_dist_integral = error_dist_integral + error_dist * dt;

v = gainStruct.Kp_v * error_dist + ...
    gainStruct.Ki_v * error_dist_integral + ...
    gainStruct.Kd_v * error_dist_derivative;

% PID for angular velocity (based on heading error)
error_heading_derivative = (error_heading - error_heading_prev) / dt;
error_heading_integral = error_heading_integral + error_heading * dt;

omega = gainStruct.Kp_omega * error_heading + ...
    gainStruct.Ki_omega * error_heading_integral + ...
    gainStruct.Kd_omega * error_heading_derivative;

% Update previous errors
error_dist_prev = error_dist;
error_heading_prev = error_heading;

% Saturate control inputs
v = max(min(v, v_max), 0);
omega = max(min(omega, omega_max), -omega_max);
end

