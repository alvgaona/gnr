function x_dot = unicycle(x, u)
    % UNICYCLE Unicycle dynamics model
    %
    % x_dot = unicycle(state, control)
    %
    % Computes the state derivative for a differential drive robot using
    % unicycle kinematics.
    %
    % Inputs:
    %   state   - Current state vector [x; y; theta] where:
    %             x     : x position (m)
    %             y     : y position (m)
    %             theta : heading angle (rad)
    %   control - Control input vector [v; omega] where:
    %             v     : linear velocity (m/s)
    %             omega : angular velocity (rad/s)
    %
    % Outputs:
    %   x_dot - State derivative [dx/dt; dy/dt; dtheta/dt]
    %
    % Unicycle Kinematics:
    %   dx/dt     = v * cos(theta)
    %   dy/dt     = v * sin(theta)
    %   dtheta/dt = omega
    %
    % Example:
    %   x = [0; 0; 0];           % Start at origin, heading east
    %   u = [1.0; 0.5];          % 1 m/s forward, 0.5 rad/s rotation
    %   x_dot = unicycle(x, u);
    %
    %   % Use with ODE solver:
    %   odefun = @(t, x) unicycle(x, u);
    %   [t, x] = ode45(odefun, [0 10], x);
    %
    %   % Discrete-time integration (Euler method):
    %   dt = 0.1;
    %   x_next = x + dt * unicycle(x, u);

    arguments
        x (3, 1) double
        u (2, 1) double
    end

    % Extract state variables
    theta = x(3);

    % Extract control inputs
    v = u(1);      % Linear velocity [m/s]
    omega = u(2);  % Angular velocity [rad/s]

    % Unicycle dynamics
    x_dot = [
        v * cos(theta);    % dx/dt
        v * sin(theta);    % dy/dt
        omega              % dtheta/dt
    ];
end
