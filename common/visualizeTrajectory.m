function visualizeTrajectory(trajectory, workspace_bounds)
%VISUALIZETRAJECTORY Plot trajectory, orientations, and workspace bounds.

    K = size(trajectory.p, 1);

    figure('Color', 'w');

    %% 3D trajectory

    subplot(2,1,1);
    hold on;
    grid on;
    box on;
    axis equal;
    view(3);

    drawWorkspace(workspace_bounds);

    plot3(trajectory.p(:,1), ...
          trajectory.p(:,2), ...
          trajectory.p(:,3), ...
          'k-', 'LineWidth', 2);

    scatter3(trajectory.p(1,1), ...
             trajectory.p(1,2), ...
             trajectory.p(1,3), ...
             70, 'g', 'filled');

    scatter3(trajectory.p(end,1), ...
             trajectory.p(end,2), ...
             trajectory.p(end,3), ...
             70, 'r', 'filled');

    %% Body coordinate frames

    frame_indices = unique(round(linspace(1, K, 8)));
    axis_length = 0.4;

    for index = frame_indices
        position = trajectory.p(index,:);
        R = trajectory.R(:,:,index);

        body_x = R(:,1)';
        body_y = R(:,2)';
        body_z = R(:,3)';

        quiver3(position(1), position(2), position(3), ...
            axis_length*body_x(1), axis_length*body_x(2), ...
            axis_length*body_x(3), 0, 'r', 'LineWidth', 1.3);

        quiver3(position(1), position(2), position(3), ...
            axis_length*body_y(1), axis_length*body_y(2), ...
            axis_length*body_y(3), 0, 'g', 'LineWidth', 1.3);

        quiver3(position(1), position(2), position(3), ...
            axis_length*body_z(1), axis_length*body_z(2), ...
            axis_length*body_z(3), 0, 'b', 'LineWidth', 1.3);
    end

    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('Prescribed Drone Trajectory');

    axis([workspace_bounds(1,:) ...
          workspace_bounds(2,:) ...
          workspace_bounds(3,:)]);

    %% Pitch history

    subplot(2,1,2);

    plot(trajectory.t, rad2deg(trajectory.pitch), ...
        'b-', 'LineWidth', 2);

    grid on;
    box on;

    xlabel('Time (s)');
    ylabel('Pitch (degrees)');
    title('Pitch During the Maneuver');

    ylim([0 95]);
end


function drawWorkspace(bounds)
%DRAWWORKSPACE Draw workspace as a wireframe cuboid.

    xmin = bounds(1,1);
    xmax = bounds(1,2);
    ymin = bounds(2,1);
    ymax = bounds(2,2);
    zmin = bounds(3,1);
    zmax = bounds(3,2);

    vertices = [
        xmin ymin zmin
        xmax ymin zmin
        xmax ymax zmin
        xmin ymax zmin
        xmin ymin zmax
        xmax ymin zmax
        xmax ymax zmax
        xmin ymax zmax
    ];

    edges = [
        1 2; 2 3; 3 4; 4 1
        5 6; 6 7; 7 8; 8 5
        1 5; 2 6; 3 7; 4 8
    ];

    for i = 1:size(edges,1)
        points = vertices(edges(i,:),:);

        plot3(points(:,1), points(:,2), points(:,3), ...
            '--', 'Color', [0.4 0.4 0.4], ...
            'LineWidth', 1);
    end
end