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

        function [scan, varargout] = scan(obj, robot_pose, environment)
            % scan - Perform a laser scan
            %
            % Args:
            %   robot_pose: Robot pose [x; y; theta] (3x1)
            %   environment: Can be:
            %     - HesseMap object
            %     - Line matrix [alpha, d] (Nx2) for Hesse lines
            %     - Cell array of obstacle structs (see below)
            %
            % Obstacle struct format (for cell arrays):
            %   .type = 'wall' | 'square' | 'circle'
            %   For 'wall': .p1 = [x1; y1], .p2 = [x2; y2] (endpoints)
            %   For 'square': .center = [x; y], .size = side_length
            %   For 'circle': .center = [x; y], .radius = r
            %
            % Returns:
            %   scan: Scan data [range, bearing] (Mx2), NaN for no return
            %   varargout{1}: ranges only (if two outputs requested)
            %   varargout{2}: bearings only (if two outputs requested)
            %
            % Example:
            %   scan = scanner.scan(pose, hesse_map);
            %   scan = scanner.scan(pose, obstacles);

            robot_pose = robot_pose(:);
            ranges = inf(size(obj.angles));

            % Determine environment type and scan accordingly
            if isa(environment, 'HesseMap')
                % HesseMap object
                ranges = obj.scanHesseLines(robot_pose, environment.getLines());
            elseif iscell(environment)
                % Cell array of obstacles
                ranges = obj.scanObstacles(robot_pose, environment);
            elseif ismatrix(environment) && size(environment, 2) == 2
                % Raw Hesse line matrix [alpha, d]
                ranges = obj.scanHesseLines(robot_pose, environment);
            else
                error('Invalid environment format. Expected HesseMap, obstacle cell array, or [alpha,d] matrix.');
            end

            % Mark out-of-range readings as NaN
            ranges(isinf(ranges)) = NaN;

            % Add measurement noise
            ranges = ranges + obj.noise_std * randn(size(ranges));

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

    methods (Access = private)
        function ranges = scanHesseLines(obj, robot_pose, map_lines)
            % scanHesseLines - Ray casting for infinite Hesse lines
            %
            % Args:
            %   robot_pose: [x; y; theta] (3x1)
            %   map_lines: [alpha, d] matrix (Nx2)
            %
            % Returns:
            %   ranges: Range measurements (1xM), inf for misses

            ranges = inf(size(obj.angles));

            % Ray casting: find intersections with infinite lines
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
        end

        function ranges = scanObstacles(obj, robot_pose, obstacles)
            % scanObstacles - Ray casting for generic obstacles
            %
            % Args:
            %   robot_pose: [x; y; theta] (3x1)
            %   obstacles: Cell array of obstacle structs
            %
            % Returns:
            %   ranges: Range measurements (1xM), inf for misses

            ranges = inf(size(obj.angles));
            rx = robot_pose(1);
            ry = robot_pose(2);
            rtheta = robot_pose(3);

            % Raycast for each beam
            for k = 1:obj.num_beams
                % Ray direction in world frame
                phi = obj.angles(k) + rtheta;
                ray_dir = [cos(phi); sin(phi)];

                % Check intersection with all obstacles
                for obs_idx = 1:length(obstacles)
                    obs = obstacles{obs_idx};

                    switch obs.type
                        case 'wall'
                            % Wall defined by two endpoints
                            t = obj.raycastSegment(rx, ry, ray_dir, obs.p1, obs.p2);
                            if t > 0 && t < obj.max_range
                                ranges(k) = min(ranges(k), t);
                            end

                        case 'square'
                            % Square defined by center and size
                            center = obs.center;
                            half_size = obs.size / 2;

                            % Four edges of square
                            corners = [
                                center(1) - half_size, center(2) - half_size;
                                center(1) + half_size, center(2) - half_size;
                                center(1) + half_size, center(2) + half_size;
                                center(1) - half_size, center(2) + half_size;
                            ];

                            % Check each edge
                            for i = 1:4
                                p1 = corners(i, :)';
                                p2 = corners(mod(i, 4) + 1, :)';
                                t = obj.raycastSegment(rx, ry, ray_dir, p1, p2);
                                if t > 0 && t < obj.max_range
                                    ranges(k) = min(ranges(k), t);
                                end
                            end

                        case 'circle'
                            % Circle defined by center and radius
                            t = obj.raycastCircle(rx, ry, ray_dir, obs.center, obs.radius);
                            if t > 0 && t < obj.max_range
                                ranges(k) = min(ranges(k), t);
                            end
                    end
                end
            end
        end

        function t = raycastSegment(~, rx, ry, ray_dir, p1, p2)
            % raycastSegment - Ray-segment intersection
            %
            % Returns distance t along ray, or -1 if no intersection

            rdx = ray_dir(1);
            rdy = ray_dir(2);

            sdx = p2(1) - p1(1);
            sdy = p2(2) - p1(2);

            % Solve: [rx, ry] + t*[rdx, rdy] = p1 + s*[sdx, sdy]
            denom = rdx * (-sdy) - rdy * (-sdx);

            if abs(denom) < 1e-6
                % Parallel or collinear
                t = -1;
                return;
            end

            % Cramer's rule
            dx = p1(1) - rx;
            dy = p1(2) - ry;

            t = (dx * (-sdy) - dy * (-sdx)) / denom;
            s = (rdx * dy - rdy * dx) / denom;

            % Check if intersection is on both ray and segment
            if s >= 0 && s <= 1 && t > 0
                % Valid intersection
                return;
            else
                t = -1;
            end
        end

        function t = raycastCircle(~, rx, ry, ray_dir, center, radius)
            % raycastCircle - Ray-circle intersection
            %
            % Returns distance t to nearest intersection, or -1 if no hit

            % Vector from ray origin to circle center
            dx = center(1) - rx;
            dy = center(2) - ry;

            % Quadratic equation: a*t^2 + b*t + c = 0
            a = ray_dir(1)^2 + ray_dir(2)^2;  % Should be 1 for normalized ray
            b = -2 * (dx * ray_dir(1) + dy * ray_dir(2));
            c = dx^2 + dy^2 - radius^2;

            discriminant = b^2 - 4*a*c;

            if discriminant < 0
                % No intersection
                t = -1;
                return;
            end

            % Two solutions (entry and exit points)
            sqrt_disc = sqrt(discriminant);
            t1 = (-b - sqrt_disc) / (2*a);
            t2 = (-b + sqrt_disc) / (2*a);

            % Return nearest positive intersection
            if t1 > 0
                t = t1;
            elseif t2 > 0
                t = t2;
            else
                t = -1;
            end
        end
    end
end
