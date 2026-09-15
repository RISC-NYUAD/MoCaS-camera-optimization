function plotCameraLayoutCube(best_layout, workspace_bounds, roi_bounds_all, voxels, showBlindSpots, blind_spots)
% plotCameraLayoutCube
% best_layout      : [N x 5] -> [x y z yaw pitch]
% workspace_bounds : [3 x 2]
% roi_bounds_all   : [3 x 2 x M]  multiple ROI boxes
% voxels           : [K x 3] optional (all voxel positions)  (existing behavior)
% showBlindSpots   : logical flag (optional). If true, plot blind spots as discrete points.
% blind_spots      : [Kbad x 3] optional. Points to plot as blind spots (e.g., voxels with g(x)<kreq)

    if nargin < 4 || isempty(voxels)
        voxels = [];
    end
    if nargin < 5 || isempty(showBlindSpots)
        showBlindSpots = false;
    end
    if nargin < 6
        blind_spots = [];
    end

    figure; hold on; grid on; axis equal; view(3);

    %% --- basic axes formatting (similar spirit to cubeWithCamerasAndTopCones) ---
    xlabel('X'); ylabel('Y'); zlabel('Z');

    % Use workspace bounds for axis & ticks
    xmin = workspace_bounds(1,1); xmax = workspace_bounds(1,2);
    ymin = workspace_bounds(2,1); ymax = workspace_bounds(2,2);
    zmin = workspace_bounds(3,1); zmax = workspace_bounds(3,2);

    xmid = (xmin + xmax)/2;
    ymid = (ymin + ymax)/2;
    zmid = (zmin + zmax)/2;

    axis([xmin xmax ymin ymax zmin zmax]);

    xticks([xmin xmid xmax]);
    yticks([ymin ymid ymax]);
    zticks([zmin zmid zmax]);

    %% --- draw workspace as a wireframe cube ---
    drawCubeWire(workspace_bounds, [0 0 0]);  % black edges, no fill

    %% --- draw ALL ROI boxes (semi-transparent colored solids) ---
    roi_colors = [
        0.90 0.40 0.40;   % soft red
        0.40 0.60 0.90;   % soft blue
        0.40 0.85 0.40;   % soft green
    ];
    if ~isempty(roi_bounds_all)
        M = size(roi_bounds_all, 3);
        for m = 1:M
            drawCubeSolid(roi_bounds_all(:,:,m), roi_colors(rem(m,3)+1,:), 0.07);
        end
    end


    %% --- NEW: plot blind spots as discrete points (on top) ---
    if showBlindSpots && ~isempty(blind_spots)
        % Make them visually salient. (You can tweak size/color if you want.)
        plot3(blind_spots(:,1), blind_spots(:,2), blind_spots(:,3), 'o', ...
              'MarkerSize', 4, ...
              'MarkerFaceColor', [1 0.2 0.2], ...
              'MarkerEdgeColor', 'none');
    end

    %% --- draw cameras (styled like cubeWithCamerasAndTopCones) ---
    N = size(best_layout,1);
    fov_deg   = 70;
    fov_rad   = deg2rad(fov_deg);
    cone_len  = 0.1 * norm(workspace_bounds(:,2) - workspace_bounds(:,1));
    cam_scale = 0.08 * norm(workspace_bounds(:,2) - workspace_bounds(:,1));

    for i = 1:N
        p     = best_layout(i,1:3);
        yaw   = best_layout(i,4);
        pitch = best_layout(i,5);

        dir = [cos(pitch)*cos(yaw), ...
               cos(pitch)*sin(yaw), ...
               sin(pitch)];
        dir = dir / norm(dir);

        drawCameraModel(p, dir, cam_scale);
        drawFOVConeFixedLength(p, dir, fov_rad, cone_len);
    end

    if showBlindSpots && ~isempty(blind_spots)
        title('Workspace, ROIs, Camera Layout + Blind Spots');
        legend_entries = {};
        if ~isempty(voxels), legend_entries{end+1} = 'Voxels'; end %#ok<AGROW>
        legend_entries{end+1} = 'Blind spots';
        legend_entries{end+1} = 'Cameras/FoV';
        % Legend for cameras is messy because they are patches; you can skip legend if it clutters.
    else
        title('Workspace, ROIs, and Optimized Camera Layout (styled cones & cameras)');
    end

    lighting gouraud; camlight;
end

%% ========================================================================
%% helper: wireframe cube (no faces, only edges; like drawCubeQuads style)
function drawCubeWire(bounds, edgeColor)
    xmin = bounds(1,1); xmax = bounds(1,2);
    ymin = bounds(2,1); ymax = bounds(2,2);
    zmin = bounds(3,1); zmax = bounds(3,2);

    V = [ xmin ymin zmin;
          xmax ymin zmin;
          xmax ymax zmin;
          xmin ymax zmin;
          xmin ymin zmax;
          xmax ymin zmax;
          xmax ymax zmax;
          xmin ymax zmax ];

    edges = [1 2; 2 3; 3 4; 4 1; ...
             5 6; 6 7; 7 8; 8 5; ...
             1 5; 2 6; 3 7; 4 8];

    for i = 1:size(edges,1)
        e = edges(i,:);
        plot3(V(e,1), V(e,2), V(e,3), 'Color', edgeColor, 'LineWidth', 1.0);
    end
end

%% helper: solid semi-transparent cube (for ROIs)
function drawCubeSolid(bounds, faceColor, faceAlpha)
    xmin = bounds(1,1); xmax = bounds(1,2);
    ymin = bounds(2,1); ymax = bounds(2,2);
    zmin = bounds(3,1); zmax = bounds(3,2);

    V = [ xmin ymin zmin;
          xmax ymin zmin;
          xmax ymax zmin;
          xmin ymax zmin;
          xmin ymin zmax;
          xmax ymin zmax;
          xmax ymax zmax;
          xmin ymax zmax ];

    F = [1 2 3 4;
         5 6 7 8;
         1 2 6 5;
         2 3 7 6;
         3 4 8 7;
         4 1 5 8];

    patch('Vertices',V,'Faces',F, ...
          'FaceColor',faceColor, ...
          'FaceAlpha',faceAlpha, ...
          'EdgeColor','none');
end

%% helper: camera model (pyramid)
function drawCameraModel(pos, dir, scale)
    dir = dir / norm(dir);
    up = [0, 0, 1];
    if abs(dot(up, dir)) > 0.99
        up = [0, 1, 0];
    end
    x = cross(up, dir); x = x / norm(x);
    y = cross(dir, x);  y = y / norm(y);
    R = [x(:), y(:), dir(:)];

    base_pts = scale * [ 0.3  0.3  1;
                        -0.3  0.3  1;
                        -0.3 -0.3  1;
                         0.3 -0.3  1];
    tip = [0, 0, 0];

    base_pts = (R * base_pts')' + pos;
    tip      = tip + pos;

    for i = 1:4
        j = mod(i,4)+1;
        fill3([tip(1), base_pts(i,1), base_pts(j,1)], ...
              [tip(2), base_pts(i,2), base_pts(j,2)], ...
              [tip(3), base_pts(i,3), base_pts(j,3)], ...
              [0.1 0.1 0.1], 'FaceAlpha', 0.7, 'EdgeColor', 'none');
    end
end

%% helper: filled FOV cone with fixed length
function drawFOVConeFixedLength(origin, direction, fov_rad, length)
    direction = direction / norm(direction);
    tip         = origin;
    base_center = tip + length * direction;
    radius      = length * tan(fov_rad / 2);

    z = direction(:);
    up = [0 0 1]';
    if abs(dot(z, up)) > 0.999
        up = [0 1 0]';
    end
    x = cross(up, z); x = x / norm(x);
    y = cross(z, x);  y = y / norm(y);
    R = [x, y, z];

    num_points   = 40;
    theta        = linspace(0, 2*pi, num_points);
    circle_local = radius * [cos(theta); sin(theta); zeros(1, num_points)];
    circle_world = R * circle_local + base_center(:);

    for i = 1:num_points-1
        verts = [tip; circle_world(:,i)'; circle_world(:,i+1)'];
        patch('Vertices', verts, 'Faces', [1 2 3], ...
              'FaceColor', [0.5 0.8 1], 'FaceAlpha', 0.3, 'EdgeColor', 'none');
    end
    verts = [tip; circle_world(:,end)'; circle_world(:,1)'];
    patch('Vertices', verts, 'Faces', [1 2 3], ...
          'FaceColor', [0.5 0.8 1], 'FaceAlpha', 0.3, 'EdgeColor', 'none');

    fill3(circle_world(1,:), circle_world(2,:), circle_world(3,:), ...
          [0.5 0.8 1], 'FaceAlpha', 0.2, 'EdgeColor', 'none');
end
