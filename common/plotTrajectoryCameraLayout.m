function plotTrajectoryCameraLayout( ...
    camera_layout, problem, trackable, figure_title)
%PLOTTRAJECTORYCAMERALAYOUT Plot cameras, FOVs, trajectory, and failures.
%
% camera_layout : [N x 5] matrix containing [x, y, z, yaw, pitch]
% problem       : optimization problem structure
% trackable     : [K x 1] logical vector; false indicates tracking failure
% figure_title  : optional figure title

    if nargin < 3
        trackable = [];
    end

    if nargin < 4 || isempty(figure_title)
        figure_title = 'Camera Layout';
    end

    if size(camera_layout, 2) ~= 5
        error('camera_layout must have size N-by-5: [x y z yaw pitch].');
    end

    workspace_bounds = problem.workspace_bounds;
    trajectory       = problem.trajectory.p;

    K = size(trajectory, 1);

    if ~isempty(trackable)
        trackable = logical(trackable(:));

        if numel(trackable) ~= K
            error(['The number of elements in trackable must match ', ...
                   'the number of trajectory samples.']);
        end
    end

    %% Create figure

    figure( ...
        'Color', 'w', ...
        'Name', figure_title);

    hold on;
    grid on;
    box on;
    axis equal;
    view(35, 25);

    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');

    %% Workspace limits

    xmin = workspace_bounds(1,1);
    xmax = workspace_bounds(1,2);

    ymin = workspace_bounds(2,1);
    ymax = workspace_bounds(2,2);

    zmin = workspace_bounds(3,1);
    zmax = workspace_bounds(3,2);

    axis([xmin xmax ymin ymax zmin zmax]);

    xticks([xmin, (xmin+xmax)/2, xmax]);
    yticks([ymin, (ymin+ymax)/2, ymax]);
    zticks([zmin, (zmin+zmax)/2, zmax]);

    drawWorkspaceWireframe(workspace_bounds);

    %% Plot rigid-body centre trajectory

    plot3( ...
        trajectory(:,1), ...
        trajectory(:,2), ...
        trajectory(:,3), ...
        'k-', ...
        'LineWidth', 2.2, ...
        'DisplayName', 'Rigid-body trajectory');

    %% Plot marker trajectories

    if isfield(problem, 'markers_world') && ...
            ~isempty(problem.markers_world)

        num_markers = size(problem.markers_world, 2);

        for marker_index = 1:num_markers

            marker_path = squeeze( ...
                problem.markers_world(:, marker_index, :));

            % Protect against squeeze changing orientation
            if size(marker_path, 2) ~= 3
                marker_path = marker_path.';
            end

            if marker_index == 1
                visibility_setting = 'on';
                marker_name = 'Marker trajectories';
            else
                visibility_setting = 'off';
                marker_name = '';
            end

            plot3( ...
                marker_path(:,1), ...
                marker_path(:,2), ...
                marker_path(:,3), ...
                '-', ...
                'Color', [0.90 0.55 0.15], ...
                'LineWidth', 1.0, ...
                'HandleVisibility', visibility_setting, ...
                'DisplayName', marker_name);
        end
    end

    %% Start and end positions

    scatter3( ...
        trajectory(1,1), ...
        trajectory(1,2), ...
        trajectory(1,3), ...
        80, ...
        [0.15 0.70 0.25], ...
        'filled', ...
        'MarkerEdgeColor', 'k', ...
        'DisplayName', 'Start');

    scatter3( ...
        trajectory(end,1), ...
        trajectory(end,2), ...
        trajectory(end,3), ...
        80, ...
        [0.75 0.20 0.75], ...
        'filled', ...
        'MarkerEdgeColor', 'k', ...
        'DisplayName', 'End');

    %% Plot failed tracking samples

    if ~isempty(trackable)

        failure_indices = find(~trackable);

        if ~isempty(failure_indices)

            failure_positions = trajectory(failure_indices, :);

            scatter3( ...
                failure_positions(:,1), ...
                failure_positions(:,2), ...
                failure_positions(:,3), ...
                45, ...
                [0.90 0.10 0.10], ...
                'filled', ...
                'MarkerEdgeColor', 'k', ...
                'DisplayName', 'Tracking failures');
        end
    end

    %% Camera visualization settings

    workspace_diagonal = norm( ...
        workspace_bounds(:,2) - workspace_bounds(:,1));

    fov_deg = problem.camera.fov_deg;
    fov_rad = deg2rad(fov_deg);

    % Visual lengths only; they do not change the camera model
    cone_length = 0.16 * workspace_diagonal;
    cam_scale   = 0.025 * workspace_diagonal;

    if isfield(problem.camera, 'max_range')
        cone_length = min(cone_length, ...
                          problem.camera.max_range);
    end

    %% Plot camera positions

    scatter3( ...
        camera_layout(:,1), ...
        camera_layout(:,2), ...
        camera_layout(:,3), ...
        65, ...
        'k', ...
        'd', ...
        'filled', ...
        'DisplayName', 'Cameras');

    %% Draw cameras and FOV cones

    num_cameras = size(camera_layout, 1);

    for camera_index = 1:num_cameras

        camera_position = camera_layout(camera_index, 1:3);
        yaw              = camera_layout(camera_index, 4);
        pitch            = camera_layout(camera_index, 5);

        camera_direction = [ ...
            cos(pitch) * cos(yaw), ...
            cos(pitch) * sin(yaw), ...
            sin(pitch)];

        camera_direction = camera_direction / ...
            norm(camera_direction);

        drawCameraModelTrajectory( ...
            camera_position, ...
            camera_direction, ...
            cam_scale);

        drawFOVConeTrajectory( ...
            camera_position, ...
            camera_direction, ...
            fov_rad, ...
            cone_length);

        text( ...
            camera_position(1), ...
            camera_position(2), ...
            camera_position(3) + 0.15, ...
            sprintf('C_%d', camera_index), ...
            'FontSize', 9, ...
            'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center', ...
            'Color', [0.1 0.1 0.1]);
    end

    %% Figure formatting

    title(sprintf('%s: %d Cameras', ...
        figure_title, num_cameras));

    legend( ...
        'show', ...
        'Location', 'best');

    set(gca, ...
        'FontSize', 11, ...
        'LineWidth', 1.0, ...
        'Projection', 'perspective');

    camlight('headlight');
    lighting gouraud;

    hold off;
end

%% ========================================================================
function drawWorkspaceWireframe(bounds)

    xmin = bounds(1,1);
    xmax = bounds(1,2);

    ymin = bounds(2,1);
    ymax = bounds(2,2);

    zmin = bounds(3,1);
    zmax = bounds(3,2);

    vertices = [ ...
        xmin ymin zmin;
        xmax ymin zmin;
        xmax ymax zmin;
        xmin ymax zmin;
        xmin ymin zmax;
        xmax ymin zmax;
        xmax ymax zmax;
        xmin ymax zmax];

    edges = [ ...
        1 2;
        2 3;
        3 4;
        4 1;
        5 6;
        6 7;
        7 8;
        8 5;
        1 5;
        2 6;
        3 7;
        4 8];

    for edge_index = 1:size(edges,1)

        edge = edges(edge_index,:);

        plot3( ...
            vertices(edge,1), ...
            vertices(edge,2), ...
            vertices(edge,3), ...
            'Color', [0.1 0.1 0.1], ...
            'LineWidth', 1.1, ...
            'HandleVisibility', 'off');
    end
end

%% ========================================================================
function drawCameraModelTrajectory(position, direction, scale)

    direction = direction / norm(direction);

    up = [0 0 1];

    if abs(dot(up, direction)) > 0.99
        up = [0 1 0];
    end

    camera_x = cross(up, direction);
    camera_x = camera_x / norm(camera_x);

    camera_y = cross(direction, camera_x);
    camera_y = camera_y / norm(camera_y);

    rotation_matrix = [ ...
        camera_x(:), ...
        camera_y(:), ...
        direction(:)];

    base_local = scale * [ ...
         0.30  0.30  1;
        -0.30  0.30  1;
        -0.30 -0.30  1;
         0.30 -0.30  1];

    base_world = ...
        (rotation_matrix * base_local.').' + position;

    tip = position;

    for side_index = 1:4

        next_index = mod(side_index, 4) + 1;

        fill3( ...
            [tip(1), ...
             base_world(side_index,1), ...
             base_world(next_index,1)], ...
            [tip(2), ...
             base_world(side_index,2), ...
             base_world(next_index,2)], ...
            [tip(3), ...
             base_world(side_index,3), ...
             base_world(next_index,3)], ...
            [0.12 0.12 0.12], ...
            'FaceAlpha', 0.85, ...
            'EdgeColor', 'none', ...
            'HandleVisibility', 'off');
    end
end

%% ========================================================================
function drawFOVConeTrajectory( ...
    origin, direction, fov_rad, cone_length)

    direction = direction / norm(direction);

    base_center = origin + cone_length * direction;
    cone_length = 1.0;
    cone_radius = cone_length * tan(fov_rad/2);

    camera_z = direction(:);
    reference_up = [0; 0; 1];

    if abs(dot(camera_z, reference_up)) > 0.999
        reference_up = [0; 1; 0];
    end

    camera_x = cross(reference_up, camera_z);
    camera_x = camera_x / norm(camera_x);

    camera_y = cross(camera_z, camera_x);
    camera_y = camera_y / norm(camera_y);

    rotation_matrix = [camera_x, camera_y, camera_z];

    num_circle_points = 40;
    theta = linspace(0, 2*pi, num_circle_points);

    circle_local = cone_radius * [ ...
        cos(theta);
        sin(theta);
        zeros(1, num_circle_points)];

    circle_world = ...
        (rotation_matrix * circle_local).' + base_center;

    cone_color = [0.35 0.70 1.00];

    for point_index = 1:num_circle_points-1

        cone_vertices = [ ...
            origin;
            circle_world(point_index,:);
            circle_world(point_index+1,:)];

        patch( ...
            'Vertices', cone_vertices, ...
            'Faces', [1 2 3], ...
            'FaceColor', cone_color, ...
            'FaceAlpha', 0.16, ...
            'EdgeColor', 'none', ...
            'HandleVisibility', 'off');
    end

    patch( ...
        'XData', circle_world(:,1), ...
        'YData', circle_world(:,2), ...
        'ZData', circle_world(:,3), ...
        'FaceColor', cone_color, ...
        'FaceAlpha', 0.08, ...
        'EdgeColor', 'none', ...
        'HandleVisibility', 'off');
end