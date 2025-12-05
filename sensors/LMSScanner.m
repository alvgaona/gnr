classdef LMSScanner < handle
    % LMSScanner - Object-oriented model of SICK LMS laser scanner
    %
    % This class models LMS200 and LMS100 laser range finders, simulating
    % their scanning behavior with configurable field of view, resolution,
    % range, and noise characteristics.
    %
    % Usage:
    %   scanner = LMSScanner('LMS200');
    %   scanner = LMSScanner('LMS100', 'MaxRange', 10, 'NoiseStd', 0.02);
    %   scan = scanner.scan(robot_pose, map);
    %   [ranges, bearings] = scanner.scan(robot_pose, map);

    properties (SetAccess = private)
        model           % Scanner model: 'LMS200' or 'LMS100'
        max_range       % Maximum sensor range [m]
        noise_std       % Range measurement noise std dev [m]
        angles          % Beam angles in robot frame [rad] (1xN)
        num_beams       % Number of laser beams
        fov             % Field of view [rad]
        angular_res     % Angular resolution [rad]
    end

    methods
        function obj = LMSScanner(model, options)
            % LMSScanner Constructor
            %
            % Args:
            %   model: Scanner model - 'LMS200' or 'LMS100'
            %   options: Name-value pairs:
            %     - MaxRange: Maximum range [m] (default: 8.0)
            %     - NoiseStd: Range noise std dev [m] (default: 0.01)
            %
            % Example:
            %   scanner = LMSScanner('LMS200');
            %   scanner = LMSScanner('LMS100', 'MaxRange', 10, 'NoiseStd', 0.02);

            arguments
                model {mustBeMember(model, {'LMS200', 'LMS100'})}
                options.MaxRange (1,1) double {mustBePositive} = 8.0
                options.NoiseStd (1,1) double {mustBeNonnegative} = 0.01
            end

            obj.model = model;
            obj.max_range = options.MaxRange;
            obj.noise_std = options.NoiseStd;

            % Configure scanner-specific parameters
            switch model
                case 'LMS200'
                    % LMS200: 180° FOV, 181 beams, 1° resolution
                    obj.fov = pi;
                    obj.num_beams = 181;
                    obj.angular_res = deg2rad(1);
                    obj.angles = linspace(-pi/2, pi/2, 181);

                case 'LMS100'
                    % LMS100: 270° FOV, 541 beams, 0.5° resolution
                    obj.fov = 3*pi/2;
                    obj.num_beams = 541;
                    obj.angular_res = deg2rad(0.5);
                    obj.angles = linspace(-3*pi/4, 3*pi/4, 541);
            end
        end

        function [scan, varargout] = scan(obj, robot_pose, map)
            % scan - Perform a laser scan
            %
            % Args:
            %   robot_pose: Robot pose [x; y; theta] (3x1)
            %   map: HesseMap object or line matrix [alpha, d] (Nx2)
            %
            % Returns:
            %   scan: Scan data [range, bearing] (Mx2), NaN for no return
            %   varargout{1}: ranges only (if two outputs requested)
            %   varargout{2}: bearings only (if two outputs requested)
            %
            % Example:
            %   scan = scanner.scan(pose, map);
            %   [ranges, bearings] = scanner.scan(pose, map);

            % Handle both HesseMap objects and raw matrices
            if isa(map, 'HesseMap')
                map_lines = map.getLines();
            else
                map_lines = map;
            end

            robot_pose = robot_pose(:);
            ranges = inf(size(obj.angles));

            % Ray casting: find intersections with map lines
            for w = 1:size(map_lines, 1)
                alpha = map_lines(w, 1);
                d_wall = map_lines(w, 2);
                n = [cos(alpha); sin(alpha)];

                % Check each laser beam for intersection
                for k = 1:obj.num_beams
                    phi = obj.angles(k) + robot_pose(3);
                    v = [cos(phi); sin(phi)];
                    denom = n' * v;

                    if abs(denom) > 1e-6
                        t = (d_wall - n' * robot_pose(1:2)) / denom;
                        if t > 0 && t < obj.max_range
                            ranges(k) = min(ranges(k), t);
                        end
                    end
                end
            end

            % Mark out-of-range readings as NaN
            ranges(isinf(ranges)) = NaN;

            % Add measurement noise to valid readings
            valid = ~isnan(ranges);
            ranges(valid) = ranges(valid) + obj.noise_std * randn(size(ranges(valid)));

            % Return format depends on number of outputs
            if nargout <= 1
                % Return [range, bearing] matrix
                scan = [ranges(:), obj.angles(:)];
            else
                % Return separate ranges and bearings
                scan = ranges(:);
                varargout{1} = obj.angles(:);
            end
        end

        function info(obj)
            % info - Display scanner specifications

            fprintf('\n========== LMS Scanner Information ==========\n');
            fprintf('Model:              %s\n', obj.model);
            fprintf('Field of View:      %.1f degrees\n', rad2deg(obj.fov));
            fprintf('Number of Beams:    %d\n', obj.num_beams);
            fprintf('Angular Resolution: %.2f degrees\n', rad2deg(obj.angular_res));
            fprintf('Maximum Range:      %.2f m\n', obj.max_range);
            fprintf('Noise Std Dev:      %.4f m\n', obj.noise_std);
            fprintf('==============================================\n\n');
        end
    end
end
