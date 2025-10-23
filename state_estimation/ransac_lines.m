function lines = ransac_lines(scan, distance_threshold, min_points)
    % Convert LIDAR scans, that are in polar coordinate,
    % to cartesian coordiantes w.r.t to the frame of the vehicle
    pts = scan(:, 1) .* [cos(scan(:, 2)), sin(scan(:, 2))];

    % Filter cartesian points that are NaN
    valid = ~isnan(scan(:, 1));
    pts = pts(valid, :);

    % Output array: [α, d, σ_α, σ_d]
    % Where α is the angle for the line,
    % d the perpendicular distance to its parallel line passing
    % through the origin
    % and (σ_α, σ_d) just the standard deviation for each parameter.
    lines = [];

    while size(pts, 1) > min_points
        best_inlier_count = 0;
        best_inliers = [];

        % RANSAC iterations
        for iter = 1:30
            % Select a random subset of the points: the minimum points
            % needed to define to fit the model (line).
            idx = randi(size(pts, 1), 2, 1);
            p1 = pts(idx(1), :);
            p2 = pts(idx(2), :);

            % Try again if points are too close
            if norm(p2 - p1) < 0.05
                continue;
            end

            % Compute the direction vector of the line
            % passing through 2 points
            dp = p2 - p1;

            % Based on the direction vector, compute the normal vector
            % by flipping the components and normalize the normal vector
            n = [-dp(2), dp(1)] / norm(dp);

            % Compute the projection of a given point (p1)
            % onto the normal vector, which is the distance
            % to the line from the origin
            d = n * p1';

            % Count inliers by calculating the projection of each
            % points to the normal vector and if that distance is greater
            % than the threshold, then it does not belong to the same line.
            % Therefore, it is an outlier. But this is just an iteration.
            in = abs(n * pts' - d) < distance_threshold;
            nin = sum(in);

            % Update best inliers
            if nin > best_inlier_count
                best_inlier_count = nin;
                best_inliers = in;
            end
        end

        % Stop if not enough inliers found
        if best_inlier_count < min_points
            break;
        end

        % Refine line using Total Least Squares (TLS) on inliers
        inlier_points = pts(best_inliers, :);
        [~, ~, V] = svd(inlier_points - mean(inlier_points, 1), 'econ');

        % Check for rank deficiency
        if size(V, 2) < 2
            pts(best_inliers, :) = [];  % Discard and continue
            continue;
        end

        % Normal is the minor axis (last singular vector)
        normal_refined = V(:, 2)';
        d_refined = normal_refined * mean(inlier_points, 1)';
        alpha_refined = atan2(normal_refined(2), normal_refined(1));

        % Estimate uncertainties from residuals
        res = normal_refined * inlier_points' - d_refined;
        sigma_d = std(res);

        % Angular uncertainty estimate based on point spread along the line
        % Project points onto the line direction (tangent)
        tangent = [-normal_refined(2), normal_refined(1)];
        projections = (inlier_points - mean(inlier_points, 1)) * tangent';
        line_extent = max(abs(projections));  % Half-length of the line segment

        % Angular uncertainty: perpendicular error / line extent
        % Ensure minimum extent to avoid division by very small numbers
        % Add conservative floor values to prevent overconfidence
        sigma_alpha = max(sigma_d / max(line_extent, 0.1), deg2rad(1.0));  % At least 1 degree
        sigma_d = max(sigma_d, 0.02);  % At least 2 cm

        % Store line parameters
        lines(end+1, :) = [alpha_refined, d_refined, sigma_alpha, sigma_d];

        % Remove inliers from point cloud
        pts(best_inliers, :) = [];
    end
end
