function map_lines = extract_map_lines(binary_map, resolution)

arguments
    binary_map (:,:) {mustBeNumeric, mustBeNonempty}
    resolution (1,1) double {mustBePositive} = 10
end

%% Edge Detection
edges = edge(binary_map, 'Canny');

%% Hough Transform
[H, theta, rho] = hough(edges);

% Find peaks in Hough space
num_peaks = 100;  % Max number of lines to detect
peaks = houghpeaks(H, num_peaks, 'threshold', ceil(0.1 * max(H(:))));
fprintf('Found %d peaks in Hough space\n', length(peaks));

% Extract lines from peaks
lines = houghlines(edges, theta, rho, peaks, 'FillGap', 20, 'MinLength', 30);
fprintf('Extracted %d line segments\n', length(lines));

%% Convert to Hesse Form and Filter Horizontal/Vertical Lines Only
map_lines = [];
angle_tolerance = deg2rad(1);  % 1 degree tolerance for horizontal/vertical

for i = 1:length(lines)
    % Hough gives theta in degrees and rho in pixels
    theta_deg = lines(i).theta;
    rho_pix = lines(i).rho;

    % Convert theta to radians
    % Hough theta: angle of perpendicular from origin to line
    % Range: [-90, 90] degrees
    theta_rad = deg2rad(theta_deg);

    % Convert to standard Hesse form [0, pi]
    if rho_pix < 0
        rho_pix = -rho_pix; % Flip to positive side
        theta_rad = theta_rad + pi;
    end

    % Normalize angle to [0, pi]
    alpha = mod(theta_rad, pi);

    % Filter: Keep only horizontal or vertical lines
    % Horizontal lines: α near 0 or π (both map to 0 in [0,π])
    % Vertical lines: α near π/2
    is_horizontal = abs(alpha) < angle_tolerance || abs(alpha - pi) < angle_tolerance;
    is_vertical = abs(alpha - pi/2) < angle_tolerance;
    
    % Skip diagonal lines
    if ~(is_horizontal || is_vertical)
        continue;
    end

    % Convert distance from pixels to meters
    d = abs(rho_pix) / resolution;

    map_lines(end+1, :) = [alpha, d];
end

map_lines = merge_similar_lines(map_lines, deg2rad(1), 0.2);
end

function merged = merge_similar_lines(lines, angle_threshold, dist_threshold)
    % Merge lines that are similar in parameter space
    if isempty(lines)
        merged = [];
        return;
    end

    % Normalize angles to [0, pi]
    lines(:, 1) = mod(lines(:, 1), pi);

    keep = true(size(lines, 1), 1);

    for i = 1:size(lines, 1)
        if ~keep(i)
            continue;
        end

        for j = i+1:size(lines, 1)
            if ~keep(j)
                continue;
            end

            % Angle difference
            angle_diff = abs(angdiff(lines(i, 1), lines(j, 1)));

            % Check flipped version too
            angle_diff_flip = abs(angdiff(lines(i, 1), mod(lines(j, 1) + pi, 2*pi)));

            angle_similar = min(angle_diff, angle_diff_flip) < angle_threshold;
            dist_similar = abs(lines(i, 2) - lines(j, 2)) < dist_threshold;

            if angle_similar && dist_similar
                % Average and merge
                lines(i, :) = (lines(i, :) + lines(j, :)) / 2;
                keep(j) = false;
            end
        end
    end

    merged = lines(keep, :);
end
