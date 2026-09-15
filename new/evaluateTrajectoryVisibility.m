function [visibility, visible_counts] = ...
    evaluateTrajectoryVisibility(layout, problem)
%EVALUATETRAJECTORYVISIBILITY Evaluate marker visibility along trajectory.
%
% Inputs:
%   layout                : N x 5 camera parameters
%                           [x, y, z, yaw, pitch]
%
%   problem.markers_world : K x M x 3 marker positions
%
% Outputs:
%   visibility            : K x M x N logical array
%   visible_counts        : K x M number of cameras viewing each marker

    %% Dimensions

    markers_world = problem.markers_world;

    K = size(markers_world, 1);
    M = size(markers_world, 2);
    N = size(layout, 1);

    if size(layout,2) ~= 5
        error('Camera layout must have size N x 5.');
    end

    if N ~= problem.num_cams
        error('Layout camera count does not match problem.num_cams.');
    end

    if size(markers_world,3) ~= 3
        error('problem.markers_world must have size K x M x 3.');
    end

    %% Camera parameters

    half_fov_angle = deg2rad(problem.camera.fov_deg / 2);

    minimum_range = problem.camera.min_range;
    maximum_range = problem.camera.max_range;

    fov_threshold = cos(half_fov_angle);

    %% Flatten marker positions for vectorized evaluation

    marker_points = reshape(markers_world, K*M, 3);

    visibility = false(K, M, N);

    %% Check every camera

    for camera_index = 1:N

        camera_position = layout(camera_index,1:3);
        camera_yaw      = layout(camera_index,4);
        camera_pitch    = layout(camera_index,5);

        % Camera optical-axis direction in the world frame
        camera_direction = [
            cos(camera_pitch) * cos(camera_yaw), ...
            cos(camera_pitch) * sin(camera_yaw), ...
            sin(camera_pitch)
        ];

        camera_direction = ...
            camera_direction / norm(camera_direction);

        %% Vectors from camera to all markers

        relative_vectors = marker_points - camera_position;

        distances = vecnorm(relative_vectors, 2, 2);

        normalized_rays = relative_vectors ./ (distances + eps);

        %% Field-of-view condition

        cosine_angles = normalized_rays * camera_direction';

        in_field_of_view = cosine_angles >= fov_threshold;

        %% Range condition

        in_range = ...
            distances >= minimum_range & ...
            distances <= maximum_range;

        %% Initial visibility

        visible_from_camera = in_field_of_view & in_range;

        visible_from_camera = ...
            reshape(visible_from_camera, K, M);

        %% Robot self-occlusion

        use_self_occlusion = ...
            isfield(problem, 'visibility') && ...
            isfield(problem.visibility, 'use_self_occlusion') && ...
            problem.visibility.use_self_occlusion;

        if use_self_occlusion
            self_occluded = checkRobotSelfOcclusion( ...
                camera_position, ...
                problem, ...
                visible_from_camera);

            if ~isequal(size(self_occluded), [K, M])
                error('Self-occlusion output must have size K x M.');
            end

            visible_from_camera = ...
                visible_from_camera & ~self_occluded;
        end

        %% Store camera visibility

        visibility(:,:,camera_index) = visible_from_camera;
    end

    %% Number of cameras observing every marker

    visible_counts = sum(visibility, 3);
end