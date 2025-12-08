function x_dot = differential_drive(x, u, L)
    % DIFFERENTIAL_DRIVE Differential drive robot dynamics model
    %
    % x_dot = differential_drive(state, control, L)
    %
    % Computes the state derivative for a differential drive robot using
    % left and right wheel velocities.
    %
    % Inputs:
    %   state   - Current state vector [x; y; theta] where:
    %             x     : x position (m)
    %             y     : y position (m)
    %             theta : heading angle (rad)
    %   control - Control input vector [v_l; v_r] where:
    %             v_l : left wheel velocity (m/s)
    %             v_r : right wheel velocity (m/s)
    %   L       - Distance between wheels (wheelbase width) (m), default = 0.5
    %
    % Outputs:
    %   x_dot - State derivative [dx/dt; dy/dt; dtheta/dt]
    %
    % Differential Drive Kinematics:
    %   v     = (v_r + v_l) / 2         (linear velocity)
    %   omega = (v_r - v_l) / L         (angular velocity)
    %
    %   dx/dt     = v * cos(theta)
    %   dy/dt     = v * sin(theta)
    %   dtheta/dt = omega
    %
    % Example:
    %   x = [0; 0; 0];           % Start at origin, heading east
    %   u = [1.0; 1.2];          % Left wheel: 1 m/s, Right wheel: 1.2 m/s
    %   L = 0.5;                 % 0.5 m wheelbase
    %   x_dot = differential_drive(x, u, L);
    %
    %   % Use with ODE solver:
    %   odefun = @(t, x) differential_drive(x, u, L);
    %   [t, x] = ode45(odefun, [0 10], x);
    %
    %   % Discrete-time integration (Euler method):
    %   dt = 0.1;
    %   x_next = x + dt * differential_drive(x, u, L);
    %
    % Note: This model converts wheel velocities to unicycle model (v, omega).
    %       For direct unicycle control, use unicycle.m instead.

    arguments
        x (3, 1) double
        u (2, 1) double
        L double = 0.5
    end

    % Extract state variables
    theta = x(3);

    % Extract control inputs
    v_l = u(1);    % Left wheel velocity [m/s]
    v_r = u(2);    % Right wheel velocity [m/s]

    % Convert to unicycle model
    v = (v_r + v_l) / 2;        % Linear velocity [m/s]
    omega = (v_r - v_l) / L;    % Angular velocity [rad/s]

    % Differential drive dynamics (same as unicycle)
    x_dot = [
        v * cos(theta);    % dx/dt
        v * sin(theta);    % dy/dt
        omega              % dtheta/dt
    ];
end
