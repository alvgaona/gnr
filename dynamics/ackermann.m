function x_dot = ackermann(x, u, L)
    % ACKERMANN Ackermann steering dynamics model
    %
    % x_dot = ackermann(state, control, L)
    %
    % Computes the state derivative for a vehicle using Ackermann steering.
    %
    % Inputs:
    %   state   - Current state vector [x; y; theta] where:
    %             x     : x position (m)
    %             y     : y position (m)
    %             theta : heading angle (rad)
    %   control - Control input vector [v; phi] where:
    %             v   : linear velocity (m/s)
    %             phi : steering angle (rad)
    %   L       - Wheelbase length (m), default = 2.5
    %
    % Outputs:
    %   x_dot - State derivative [dx/dt; dy/dt; dtheta/dt]
    %
    % Ackermann Steering Kinematics:
    %   dx/dt     = v * cos(theta)
    %   dy/dt     = v * sin(theta)
    %   dtheta/dt = (v/L) * tan(phi)
    %
    % Example:
    %   x = [0; 0; 0];           % Start at origin, heading east
    %   u = [1.0; 0.1];        % 1 m/s forward, 0.1 rad steering
    %   L = 2.5;                     % 2.5 m wheelbase
    %   x_dot = ackermann(state, control, L);
    %
    %   % Use with ODE solver:
    %   odefun = @(t, x) ackermann(x, control, L);
    %   [t, x] = ode45(odefun, [0 10], state);

    arguments
        x (3, 1) double
        u (2, 1) double
        L double = 2.5
    end

    % Extract state variables
    theta = x(3);

    % Extract control inputs
    v = u(1);      % Linear velocity [m/s]
    phi = u(2);    % Steering angle [rad]

    % Ackermann steering dynamics
    x_dot = [
        v * cos(theta);           % dx/dt
        v * sin(theta);           % dy/dt
        (v / L) * tan(phi)        % dtheta/dt (angular velocity)
    ];
end
