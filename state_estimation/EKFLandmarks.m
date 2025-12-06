classdef EKFLandmarks < handle
    % EKFLandmarks - Extended Kalman Filter for Landmark-based Localization
    properties
        x           % State estimate [x; y; theta] (3x1)
        P           % State covariance matrix (3x3)
        landmarks   % Landmark positions [x, y] (Nx2)
        Q           % Process noise covariance [Δd; Δβ] (2x2)
    end

    methods
        function obj = EKFLandmarks(initial_state, initial_covariance, ...
                                    landmarks, process_noise_cov)
            % Constructor
            obj.x = initial_state(:);
            obj.P = initial_covariance;
            obj.landmarks = landmarks;
            obj.Q = process_noise_cov;
        end

        function predict(obj, delta_d, delta_theta)
            % predict - EKF prediction step using odometry
            theta_k = obj.x(3);

            % Midpoint odometry model
            theta_mid = theta_k + delta_theta / 2;
            obj.x = [
                obj.x(1) + delta_d * cos(theta_mid);
                obj.x(2) + delta_d * sin(theta_mid);
                theta_k + delta_theta
            ];

            % Normalize heading angle
            obj.x(3) = atan2(sin(obj.x(3)), cos(obj.x(3)));

            % Jacobian of transition function with respect to state
            % f(x,y,θ) = [x + Δd*cos(θ + Δβ/2); y + Δd*sin(θ + Δβ/2); θ + Δβ]
            A = [
                1, 0, -delta_d * sin(theta_mid);
                0, 1,  delta_d * cos(theta_mid);
                0, 0,  1
            ];

            % Jacobian with respect to odometry input u = [Δd; Δβ]
            W = [
                cos(theta_mid),  -0.5 * delta_d * sin(theta_mid);
                sin(theta_mid),   0.5 * delta_d * cos(theta_mid);
                0,                1
            ];

            % Predict covariance
            obj.P = A * obj.P * A' + W * obj.Q * W';
        end

        function update(obj, measurements, R)
            % update - EKF update step using landmark observations
            num_landmarks = size(obj.landmarks, 1);
            num_measurements = length(measurements);

            % Determine if we have range or bearing-only
            has_range = (num_measurements == 2 * num_landmarks);

            % Compute predicted measurements - Vectorized
            dx_pred = obj.landmarks(:,1) - obj.x(1);
            dy_pred = obj.landmarks(:,2) - obj.x(2);
            dist_sq = dx_pred.^2 + dy_pred.^2;
            ranges_pred = sqrt(dist_sq);

            % Predicted bearings (relative to robot heading)
            predicted_bearings = atan2(dy_pred, dx_pred);
            predicted_relative_bearings = predicted_bearings - obj.x(3);

            % Normalize to [-pi, pi]
            predicted_relative_bearings = atan2(sin(predicted_relative_bearings), ...
                                                cos(predicted_relative_bearings));

            if has_range
                % Range-and-bearing measurements: [r1, phi1, r2, phi2, ...]
                % Build predicted measurement vector
                predicted_measurements = zeros(num_measurements, 1);
                predicted_measurements(1:2:end) = ranges_pred;
                predicted_measurements(2:2:end) = predicted_relative_bearings;

                % Normalize bearing predictions
                predicted_measurements(2:2:end) = atan2(sin(predicted_measurements(2:2:end)), ...
                                                        cos(predicted_measurements(2:2:end)));

                % Measurement Jacobian H (2Nx3 matrix)
                H = zeros(num_measurements, 3);
                for i = 1:num_landmarks
                    row_range = 2*i - 1;
                    row_bearing = 2*i;

                    % Range measurement Jacobian
                    H(row_range, 1) = -dx_pred(i) / ranges_pred(i);  % ∂r/∂x
                    H(row_range, 2) = -dy_pred(i) / ranges_pred(i);  % ∂r/∂y
                    H(row_range, 3) = 0;                             % ∂r/∂θ

                    % Bearing measurement Jacobian
                    H(row_bearing, 1) = dy_pred(i) / dist_sq(i);     % ∂φ/∂x
                    H(row_bearing, 2) = -dx_pred(i) / dist_sq(i);    % ∂φ/∂y
                    H(row_bearing, 3) = -1;                          % ∂φ/∂θ
                end

                % Innovation with angle normalization
                innovation = measurements - predicted_measurements;
                innovation(2:2:end) = atan2(sin(innovation(2:2:end)), cos(innovation(2:2:end)));
            else
                % Bearing-only measurements: [phi1; phi2; ...]
                predicted_measurements = predicted_relative_bearings;

                % Measurement Jacobian (Nx3 matrix)
                H = [dy_pred ./ dist_sq, -dx_pred ./ dist_sq, -ones(num_landmarks, 1)];

                % Innovation with angle normalization
                innovation = measurements - predicted_measurements;
                innovation = atan2(sin(innovation), cos(innovation));
            end

            % Innovation covariance
            S = H * obj.P * H' + R;

            % Kalman gain
            K = obj.P * H' / S;

            % Update state estimate
            obj.x = obj.x + K * innovation;

            % Normalize theta to [-pi, pi]
            obj.x(3) = atan2(sin(obj.x(3)), cos(obj.x(3)));

            % Update covariance
            obj.P = (eye(3) - K * H) * obj.P;
        end
    end
end
