function [objective_value, terms] = ...
    trajectoryLayoutObjective(x, problem)
%TRAJECTORYLAYOUTOBJECTIVE Objective for trajectory-aware camera placement.
%
% Each camera is represented by:
%   [x, y, z, yaw, pitch]

    num_cams = problem.num_cams;
    k_req    = problem.k_req;
    M_req    = problem.M_req;

    %% Reshape decision vector

    layout = reshape(x, [5, num_cams])';

    camera_positions = layout(:,1:3);
    camera_yaw       = layout(:,4);
    camera_pitch     = layout(:,5);

    %% Evaluate trajectory-dependent visibility

    [visibility, visible_counts] = ...
        evaluateTrajectoryVisibility(layout, problem);

    % visible_counts: K x M
    % visibility   : K x M x N

    [K, M] = size(visible_counts);

    if M_req > M
        error('M_req cannot be larger than the number of markers.');
    end

    if k_req > num_cams
        error('k_req cannot be larger than the number of cameras.');
    end

    %% 1. Rigid-body tracking penalty

    % Normalized camera-view deficiency for each marker
    marker_deficit = max(0, k_req - visible_counts) / k_req;

    % Find the M_req markers closest to satisfying the requirement
    sorted_deficit = sort(marker_deficit, 2, 'ascend');

    selected_deficit = sorted_deficit(:,1:M_req);

    % Tracking deficiency at every trajectory sample
    sample_deficit = mean(selected_deficit.^2, 2);

    if isfield(problem, 'objective') && ...
       isfield(problem.objective, 'beta')
        beta = problem.objective.beta;
    else
        beta = 0.5;
    end

    mean_tracking_deficit  = mean(sample_deficit);
    worst_tracking_deficit = max(sample_deficit);

    P_track = beta * mean_tracking_deficit + ...
             (1 - beta) * worst_tracking_deficit;

    %% 2. Inter-camera spacing penalty

    if isfield(problem.camera, 'min_spacing')
        minimum_spacing = problem.camera.min_spacing;
    else
        minimum_spacing = 2;
    end

    P_dist = 0;
    number_pairs = 0;

    for i = 1:num_cams-1
        for j = i+1:num_cams
            camera_distance = norm( ...
                camera_positions(i,:) - camera_positions(j,:));

            violation = max(0, minimum_spacing - camera_distance);

            P_dist = P_dist + violation^2;
            number_pairs = number_pairs + 1;
        end
    end

    if number_pairs > 0
        P_dist = P_dist / number_pairs;
    end

    %% 3. Trajectory-dependent stereo-geometry penalty

    if isfield(problem.camera, 'alpha_min')
        alpha_min = problem.camera.alpha_min;
    else
        alpha_min = deg2rad(10);
    end

    if isfield(problem.camera, 'alpha_max')
        alpha_max = problem.camera.alpha_max;
    else
        alpha_max = deg2rad(170);
    end

    marker_points = reshape(problem.markers_world, [], 3);
    visibility_flat = reshape(visibility, [], num_cams);

    number_points = size(marker_points, 1);

    viewing_rays = zeros(number_points, 3, num_cams);

    for i = 1:num_cams
        rays = marker_points - camera_positions(i,:);
        ray_norms = vecnorm(rays, 2, 2);

        viewing_rays(:,:,i) = rays ./ (ray_norms + eps);
    end

    stereo_penalty_sum = 0;
    stereo_sample_count = 0;

    for i = 1:num_cams-1
        for j = i+1:num_cams

            mutually_visible = ...
                visibility_flat(:,i) & visibility_flat(:,j);

            if ~any(mutually_visible)
                continue;
            end

            rays_i = viewing_rays(mutually_visible,:,i);
            rays_j = viewing_rays(mutually_visible,:,j);

            cosine_angle = sum(rays_i .* rays_j, 2);
            cosine_angle = max(-1, min(1, cosine_angle));

            stereo_angles = acos(cosine_angle);

            lower_violation = max(0, alpha_min - stereo_angles);
            upper_violation = max(0, stereo_angles - alpha_max);

            stereo_penalty_sum = stereo_penalty_sum + ...
                sum(lower_violation.^2 + upper_violation.^2);

            stereo_sample_count = stereo_sample_count + ...
                numel(stereo_angles);
        end
    end

    if stereo_sample_count > 0
        P_stereo = stereo_penalty_sum / stereo_sample_count;
    else
        P_stereo = 0;
    end

    %% 4. Inward-looking orientation penalty

    trajectory_center = mean(problem.trajectory.p, 1);

    P_center = 0;

    for i = 1:num_cams

        camera_direction = [
            cos(camera_pitch(i)) * cos(camera_yaw(i)), ...
            cos(camera_pitch(i)) * sin(camera_yaw(i)), ...
            sin(camera_pitch(i))
        ];

        direction_to_trajectory = ...
            trajectory_center - camera_positions(i,:);

        direction_norm = norm(direction_to_trajectory);

        if direction_norm < 1e-12
            continue;
        end

        direction_to_trajectory = ...
            direction_to_trajectory / direction_norm;

        alignment = dot( ...
            camera_direction, direction_to_trajectory);

        violation = max(0, -alignment);

        P_center = P_center + violation^2;
    end

    P_center = P_center / num_cams;

    %% 5. Objective weights

    weights.track  = 1;
    weights.dist   = 0.1;
    weights.stereo = 0.1;
    weights.center = 0.1;

    if isfield(problem, 'objective')
        supplied_weights = problem.objective;

        if isfield(supplied_weights, 'lambda_track')
            weights.track = supplied_weights.lambda_track;
        end

        if isfield(supplied_weights, 'lambda_dist')
            weights.dist = supplied_weights.lambda_dist;
        end

        if isfield(supplied_weights, 'lambda_stereo')
            weights.stereo = supplied_weights.lambda_stereo;
        end

        if isfield(supplied_weights, 'lambda_center')
            weights.center = supplied_weights.lambda_center;
        end
    end

    %% Final objective

    objective_value = ...
          weights.track  * P_track ...
        + weights.dist   * P_dist ...
        + weights.stereo * P_stereo ...
        + weights.center * P_center;

    %% Optional diagnostic outputs

    if nargout > 1
        terms.P_track  = P_track;
        terms.P_dist   = P_dist;
        terms.P_stereo = P_stereo;
        terms.P_center = P_center;

        terms.mean_tracking_deficit  = mean_tracking_deficit;
        terms.worst_tracking_deficit = worst_tracking_deficit;
    end
end