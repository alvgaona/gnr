classdef NMPCController < handle
    % NMPCController - Nonlinear MPC
    
    properties (SetAccess = private)
        nlobj           % NMPC instance

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

        % Optimization settings
        opt_options     % fmincon options

        % State for warm-starting
        last_u          % Last control input (2x1)
    end

    methods
        function obj = NMPCController(options)
            % NMPCController Constructor
            arguments
                options.Nx (1,1) double {mustBePositive} = 3;
                options.Ny (1,1) double {mustBePositive} = 3;
                options.Nu (1,1) double {mustBePositive} = 2;
                options.PredictionHorizon (1,1) double {mustBePositive} = 10
                options.ControlHorizon (1,1) double {mustBePositive} = 5
                options.TimeStep (1,1) double {mustBePositive} = 0.1
                options.StateWeights (1,3) double = [1, 1, 0.1]
                options.ControlWeights (1,2) double = [1, 1]
                options.VelocityLimits (1,2) double = [0, 1.5]
                options.AngularLimits (1,2) double = [-2, 2]
            end
            
            obj.N = options.PredictionHorizon;
            obj.dt = options.TimeStep;
            obj.Q = diag(options.StateWeights);
            obj.R = diag(options.ControlWeights);

            obj.v_min = options.VelocityLimits(1);
            obj.v_max = options.VelocityLimits(2);
            obj.omega_min = options.AngularLimits(1);
            obj.omega_max = options.AngularLimits(2);
            
            obj.nlobj = nlmpc(options.Nx, options.Ny, options.Nu);

            obj.nlobj.Ts = options.TimeStep;
            obj.nlobj.PredictionHorizon = options.PredictionHorizon;
            obj.nlobj.ControlHorizon = options.ControlHorizon;

            obj.nlobj.Model.StateFcn = @(x,u) [u(1)*cos(x(3));
                u(1)*sin(x(3));
                u(2)];

            obj.nlobj.MV(1).Min = options.VelocityLimits(1); % v >= v_min [m/s]
            obj.nlobj.MV(1).Max = options.VelocityLimits(2); % v <= v_max [m/s]
            obj.nlobj.MV(2).Min = options.AngularLimits(1);  % omega >= omega_min [rad/s]
            obj.nlobj.MV(2).Max = options.AngularLimits(2);  % omega <= omega_max [rad/s]

            % Weights
            obj.nlobj.Weights.OutputVariables = diag(options.StateWeights);
            obj.nlobj.Weights.ManipulatedVariables = diag(options.ControlWeights);

            obj.last_u = [0; 0];
        end

        function u = compute(obj, x, xref)
            x = x(:);

            % Trajectory provided - use first N+1 points
            xref_horizon = xref(1:min(obj.N+1, size(xref,1)), :);

            % Pad if needed
            if size(xref_horizon, 1) < obj.N + 1
                xref_horizon = [xref_horizon; repmat(xref_horizon(end,:), ...
                    obj.N + 1 - size(xref_horizon, 1), 1)];
            end

            u = nlmpcmove(obj.nlobj, x, obj.last_u, xref_horizon);
            obj.last_u = u;
        end

        function reset(obj)
            obj.last_u = [0; 0];
        end
    end
end
