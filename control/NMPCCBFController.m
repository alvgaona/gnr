classdef NMPCCBFController < handle
    % NMPCCBFController - Nonlinear MPC with Control Barrier Functions
    
    properties (SetAccess = private)
        % NMPC parameters
        N               % Prediction horizon length
        dt              % Time step [s]
        Q               % State tracking weights (3x3)
        R               % Control effort weights (2x2)

        % Control constraints
        v_min           % Minimum linear velocity [m/s]
        v_max           % Maximum linear velocity [m/s]
        omega_min       % Minimum angular velocity [rad/s]
        omega_max       % Maximum angular velocity [rad/s]

        % CBF parameters
        d_safe          % Safety radius [m]
        alpha_cbf       % CBF aggressiveness parameter
        scan_downsample % Downsample factor for scan constraints
        constraint_range % Maximum range for CBF constraints [m]

        % Optimization settings
        opt_options     % fmincon options

        % State for warm-starting
        last_u          % Last control input (2x1)
    end

    methods
        function obj = NMPCCBFController(options)
            % NMPCCBFController Constructor
            arguments
                options.HorizonLength (1,1) double {mustBePositive} = 20
                options.TimeStep (1,1) double {mustBePositive} = 0.1
                options.StateWeights (1,3) double = [1, 1, 0.1]
                options.ControlWeights (1,2) double = [1, 1]
                options.VelocityLimits (1,2) double = [0, 1.5]
                options.AngularLimits (1,2) double = [-2, 2]
                options.SafetyRadius (1,1) double {mustBePositive} = 0.6
                options.AlphaCBF (1,1) double {mustBePositive} = 1.0
                options.ScanDownsample (1,1) double {mustBePositive, mustBeInteger} = 50
                options.ConstraintRange (1,1) double {mustBePositive} = 2.0
                options.MaxIterations (1,1) double {mustBePositive} = 100
            end

            obj.N = options.HorizonLength;
            obj.dt = options.TimeStep;
            obj.Q = diag(options.StateWeights);
            obj.R = diag(options.ControlWeights);

            obj.v_min = options.VelocityLimits(1);
            obj.v_max = options.VelocityLimits(2);
            obj.omega_min = options.AngularLimits(1);
            obj.omega_max = options.AngularLimits(2);

            obj.d_safe = options.SafetyRadius;
            obj.alpha_cbf = options.AlphaCBF;
            obj.scan_downsample = options.ScanDownsample;
            obj.constraint_range = options.ConstraintRange;

            obj.opt_options = optimoptions('fmincon', ...
                'Algorithm', 'sqp', ...
                'Display', 'off', ...
                'MaxIterations', options.MaxIterations);

            obj.last_u = [0; 0];
        end

        function u = compute(obj, x, xref, scan)
            % compute - Compute control signal
            x = x(:);
            
            % Trajectory provided - use first N+1 points
            xref_horizon = xref(1:min(obj.N+1, size(xref,1)), :);

            % Pad if needed
            if size(xref_horizon, 1) < obj.N + 1
                xref_horizon = [xref_horizon; repmat(xref_horizon(end,:), ...
                    obj.N + 1 - size(xref_horizon, 1), 1)];
            end

            % Initial guess (warm start with last control)
            u0 = repmat(obj.last_u', obj.N, 1);
            u0 = u0(:);

            % Lower and upper bound for the control signal
            lb = repmat([obj.v_min; obj.omega_min], obj.N, 1);
            ub = repmat([obj.v_max; obj.omega_max], obj.N, 1);

            % Solve optimization problem
            u_opt = fmincon(@(u) obj.costFunction(u, x, xref_horizon), ...
                u0, [], [], [], [], lb, ub, ...
                @(u) obj.cbfConstraints(u, x, scan), ...
                obj.opt_options);

            % Extract first and ignore horizon control signals
            u = u_opt(1:2);

            obj.last_u = u;
        end

        function reset(obj)
            obj.last_u = [0; 0];
        end
    end

    methods (Access = private)
        function J = costFunction(obj, u_seq, x0, xref)
            u_seq = reshape(u_seq, 2, obj.N)';
            x = x0;
            J = 0;

            for k = 1:obj.N
                u = u_seq(k, :)';

                % State tracking error
                x_err = x - xref(k, :)';
                J = J + x_err' * obj.Q * x_err;

                % Control effort
                J = J + u' * obj.R * u;

                % Propagate unicycle dynamics
                x = x + obj.dt * [u(1)*cos(x(3)); u(1)*sin(x(3)); u(2)];
            end

            % Terminal cost
            x_err = x - xref(obj.N+1, :)';
            J = J + x_err' * obj.Q * x_err;
        end

        function [c, ceq] = cbfConstraints(obj, u_seq, x0, scan)
            u_seq = reshape(u_seq, 2, obj.N)';

            % Filter scan: valid readings within constraint range
            valid = ~isnan(scan(:,1)) & scan(:,1) < obj.constraint_range;

            % Downsample to reduce number of constraints
            valid_idx = find(valid);
            if isempty(valid_idx)
                % No obstacles nearby - no constraints
                c = [];
                ceq = [];
                return;
            end

            valid_idx = valid_idx(1:obj.scan_downsample:end);
            ranges = scan(valid_idx, 1);
            bearings = scan(valid_idx, 2);

            c = [];

            % Apply CBF constraint at each prediction step
            x = x0;
            for k = 1:obj.N
                u = u_seq(k, :)';
                v = u(1);
                omega = u(2);

                % Barrier function: h = range - d_safe
                h = ranges - obj.d_safe;

                % Time derivative of barrier function
                % For static obstacles: range_dot = -v*cos(bearing) - range*omega*sin(bearing)
                h_dot = -v * cos(bearings) - ranges .* omega .* sin(bearings);

                % CBF constraint: h_dot + alpha*h >= 0
                % Convert to c <= 0 format: -(h_dot + alpha*h) <= 0
                c_k = -(h_dot + obj.alpha_cbf * h);

                c = [c; c_k];

                % Propagate dynamics for next step
                x = x + obj.dt * [u(1)*cos(x(3)); u(1)*sin(x(3)); u(2)];
            end

            ceq = [];  % No equality constraints
        end
    end
end
