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
        slack_penalty   % Penalty weight for constraint violations
        use_slack       % Enable slack variables for soft constraints

        % CasADi/IPOPT settings
        solver_options  % IPOPT options
        max_iterations  % Maximum solver iterations

        % State for warm-starting
        last_u          % Last control input (2xN)
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
                options.UseSlack (1,1) logical = false
                options.SlackPenalty (1,1) double {mustBePositive} = 1000
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
            obj.use_slack = options.UseSlack;
            obj.slack_penalty = options.SlackPenalty;

            % Configure IPOPT solver options
            obj.solver_options = struct();
            obj.solver_options.ipopt.print_level = 0;
            obj.solver_options.ipopt.max_iter = options.MaxIterations;
            obj.solver_options.ipopt.acceptable_tol = 1e-6;
            obj.solver_options.ipopt.acceptable_obj_change_tol = 1e-6;
            obj.solver_options.print_time = false;
            obj.solver_options.verbose = false;

            obj.last_u = zeros(2, obj.N);
        end

        function u = compute(obj, x, xref, scan)
            % compute - Compute control signal using CasADi Opti
            import casadi.*

            x = x(:);

            % Trajectory provided - use first N+1 points
            xref_horizon = xref(1:min(obj.N+1, size(xref,1)), :);

            % Pad if needed
            if size(xref_horizon, 1) < obj.N + 1
                xref_horizon = [xref_horizon; repmat(xref_horizon(end,:), ...
                    obj.N + 1 - size(xref_horizon, 1), 1)];
            end

            % Filter scan: valid readings within constraint range
            valid = ~isnan(scan(:,1)) & scan(:,1) < obj.constraint_range;
            valid_idx = find(valid);

            if ~isempty(valid_idx)
                valid_idx = valid_idx(1:obj.scan_downsample:end);
                ranges = scan(valid_idx, 1);
                bearings = scan(valid_idx, 2);
                n_obstacles = length(ranges);
            else
                ranges = [];
                bearings = [];
                n_obstacles = 0;
            end

            % Create CasADi optimization problem
            opti = casadi.Opti();

            % Decision variables: control inputs over horizon
            U = opti.variable(2, obj.N);  % [v; omega] for each time step

            % State trajectory (for constraint evaluation)
            X = opti.variable(3, obj.N+1);

            % Initial condition constraint
            opti.subject_to(X(:,1) == x);

            % Dynamics constraints
            for k = 1:obj.N
                x_next = X(:,k) + obj.dt * [U(1,k)*cos(X(3,k));
                                             U(1,k)*sin(X(3,k));
                                             U(2,k)];
                opti.subject_to(X(:,k+1) == x_next);
            end

            % Control constraints
            opti.subject_to(obj.v_min <= U(1,:) <= obj.v_max);
            opti.subject_to(obj.omega_min <= U(2,:) <= obj.omega_max);

            % CBF constraints
            if n_obstacles > 0
                if obj.use_slack
                    % Create slack variables for soft constraints
                    % One slack per obstacle per time step
                    S = opti.variable(n_obstacles, obj.N);

                    % Slack must be non-negative (element-wise)
                    for i = 1:n_obstacles
                        for k = 1:obj.N
                            opti.subject_to(S(i,k) >= 0);
                        end
                    end

                    for k = 1:obj.N
                        v = U(1,k);
                        omega = U(2,k);

                        % Barrier function: h = range - d_safe
                        h = ranges - obj.d_safe;

                        % Time derivative of barrier function
                        h_dot = -v * cos(bearings) - ranges .* omega .* sin(bearings);

                        % Soft CBF constraint: h_dot + alpha*h + slack >= 0 (element-wise)
                        for i = 1:n_obstacles
                            opti.subject_to(h_dot(i) + obj.alpha_cbf * h(i) + S(i,k) >= 0);
                        end
                    end
                else
                    % Hard constraints (original formulation)
                    for k = 1:obj.N
                        v = U(1,k);
                        omega = U(2,k);

                        % Barrier function: h = range - d_safe
                        h = ranges - obj.d_safe;

                        % Time derivative of barrier function
                        h_dot = -v * cos(bearings) - ranges .* omega .* sin(bearings);

                        % CBF constraint: h_dot + alpha*h >= 0 (element-wise)
                        for i = 1:n_obstacles
                            opti.subject_to(h_dot(i) + obj.alpha_cbf * h(i) >= 0);
                        end
                    end
                end
            end

            % Cost function
            cost = 0;
            for k = 1:obj.N
                % State tracking error
                x_err = X(:,k) - xref_horizon(k,:)';
                cost = cost + x_err' * obj.Q * x_err;

                % Control effort
                cost = cost + U(:,k)' * obj.R * U(:,k);
            end

            % Add slack penalty to cost if using soft constraints
            if n_obstacles > 0 && obj.use_slack
                % Penalize constraint violations (sum of all slacks)
                cost = cost + obj.slack_penalty * sum(sum(S.^2));
            end

            % Terminal cost
            x_err = X(:,obj.N+1) - xref_horizon(obj.N+1,:)';
            cost = cost + x_err' * obj.Q * x_err;

            opti.minimize(cost);

            % Set initial guess (warm start)
            opti.set_initial(U, obj.last_u);

            % Set solver
            opti.solver('ipopt', obj.solver_options);

            % Solve
            try
                sol = opti.solve();
                u_opt = sol.value(U);
                obj.last_u = u_opt;  % Store for warm start
                u = u_opt(:,1);  % Return first control
            catch
                warning('NMPC optimization failed, using last solution');
                u = obj.last_u(:,1);
            end
        end

        function reset(obj)
            obj.last_u = zeros(2, obj.N);
        end
    end
end
