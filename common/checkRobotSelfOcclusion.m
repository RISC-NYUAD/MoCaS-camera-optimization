function self_occluded = checkRobotSelfOcclusion( ...
    camera_position_world, problem, candidate_visibility)
%CHECKROBOTSELFOCCLUSION Check camera-marker segments against the robot STL.
%
% The mesh BVH is traversed for each candidate segment. This avoids the
% previous O(number_of_rays * number_of_triangles) full-mesh scan.

    trajectory   = problem.trajectory;
    markers_body = problem.markers_body;
    mesh_data    = problem.occlusion_mesh;

    K = size(trajectory.p,1);
    M = size(markers_body,1);

    if nargin < 3
        candidate_visibility = true(K,M);
    end

    if ~isequal(size(candidate_visibility), [K,M])
        error('candidate_visibility must have size K x M.');
    end

    if ~isfield(mesh_data, 'bvh')
        error(['problem.occlusion_mesh does not contain a BVH. ', ...
               'Rebuild it using prepareOcclusionMesh.']);
    end

    if isfield(problem, 'visibility') && ...
            isfield(problem.visibility, 'marker_neighborhood')
        marker_neighborhood = problem.visibility.marker_neighborhood;
    else
        marker_neighborhood = 0.025;
    end

    self_occluded = false(K,M);

    if ~any(candidate_visibility(:))
        return;
    end

    %% Transform the fixed world camera into the moving body frame

    camera_positions_body = zeros(K,3);

    for sample_index = 1:K
        world_offset = ...
            camera_position_world - trajectory.p(sample_index,:);

        % Equivalent to (R_body_to_world' * world_offset')'.
        camera_positions_body(sample_index,:) = ...
            world_offset * trajectory.R(:,:,sample_index);
    end

    %% Build all candidate rays for this camera

    candidate_linear_indices = find(candidate_visibility);
    [sample_indices, marker_indices] = ind2sub( ...
        [K, M], candidate_linear_indices);

    segment_starts = camera_positions_body(sample_indices,:);
    segment_ends   = markers_body(marker_indices,:);

    number_rays = numel(candidate_linear_indices);
    ray_is_occluded = false(number_rays,1);

    for ray_index = 1:number_rays
        ray_is_occluded(ray_index) = segmentIntersectsBVH( ...
            segment_starts(ray_index,:), ...
            segment_ends(ray_index,:), ...
            mesh_data, ...
            marker_neighborhood);
    end

    self_occluded(candidate_linear_indices) = ray_is_occluded;
end


function intersects = segmentIntersectsBVH( ...
    segment_start, segment_end, mesh_data, marker_neighborhood)
%SEGMENTINTERSECTSBVH Traverse the BVH and test triangles in reached leaves.

    direction_tolerance = 1e-12;
    triangle_tolerance  = 1e-9;

    segment_vector = segment_end - segment_start;
    segment_length = norm(segment_vector);

    if segment_length <= marker_neighborhood
        intersects = false;
        return;
    end

    ray_direction = segment_vector / segment_length;

    % Hits in this final interval belong to the marker mounting region and
    % are deliberately ignored.
    maximum_hit_distance = segment_length - marker_neighborhood;

    bvh = mesh_data.bvh;

    [root_hit, ~] = segmentIntersectsBox( ...
        segment_start, ray_direction, maximum_hit_distance, ...
        bvh.node_min(bvh.root,:), bvh.node_max(bvh.root,:), ...
        direction_tolerance);

    if ~root_hit
        intersects = false;
        return;
    end

    % During depth-first traversal, the stack needs at most one deferred
    % sibling per tree level.
    node_stack = zeros(bvh.max_depth + 2, 1, 'uint32');
    stack_size = 1;
    node_stack(1) = bvh.root;

    while stack_size > 0
        node_index = node_stack(stack_size);
        stack_size = stack_size - 1;

        number_triangles = double(bvh.triangle_count(node_index));

        if number_triangles > 0
            first_triangle = double(bvh.triangle_start(node_index));
            triangle_indices = ...
                first_triangle:first_triangle + number_triangles - 1;

            if rayIntersectsTriangles( ...
                    segment_start, ray_direction, maximum_hit_distance, ...
                    mesh_data.v0(triangle_indices,:), ...
                    mesh_data.edge1(triangle_indices,:), ...
                    mesh_data.edge2(triangle_indices,:), ...
                    triangle_tolerance)
                intersects = true;
                return;
            end

            continue;
        end

        left_node  = bvh.left_child(node_index);
        right_node = bvh.right_child(node_index);

        [hit_left, near_left] = segmentIntersectsBox( ...
            segment_start, ray_direction, maximum_hit_distance, ...
            bvh.node_min(left_node,:), bvh.node_max(left_node,:), ...
            direction_tolerance);

        [hit_right, near_right] = segmentIntersectsBox( ...
            segment_start, ray_direction, maximum_hit_distance, ...
            bvh.node_min(right_node,:), bvh.node_max(right_node,:), ...
            direction_tolerance);

        % Push the farther child first so the nearer child is processed
        % immediately. An early triangle hit then terminates the traversal.
        if hit_left && hit_right
            if near_left <= near_right
                stack_size = stack_size + 1;
                node_stack(stack_size) = right_node;
                stack_size = stack_size + 1;
                node_stack(stack_size) = left_node;
            else
                stack_size = stack_size + 1;
                node_stack(stack_size) = left_node;
                stack_size = stack_size + 1;
                node_stack(stack_size) = right_node;
            end
        elseif hit_left
            stack_size = stack_size + 1;
            node_stack(stack_size) = left_node;
        elseif hit_right
            stack_size = stack_size + 1;
            node_stack(stack_size) = right_node;
        end
    end

    intersects = false;
end


function [hit, t_near] = segmentIntersectsBox( ...
    ray_origin, ray_direction, maximum_distance, ...
    box_min, box_max, tolerance)
%SEGMENTINTERSECTSBOX Slab test restricted to the useful ray segment.

    t_near = 0;
    t_far  = maximum_distance;

    for axis_index = 1:3
        direction_component = ray_direction(axis_index);

        if abs(direction_component) <= tolerance
            if ray_origin(axis_index) < box_min(axis_index) - tolerance || ...
                    ray_origin(axis_index) > box_max(axis_index) + tolerance
                hit = false;
                return;
            end
        else
            inverse_direction = 1 / direction_component;

            t1 = (box_min(axis_index) - ray_origin(axis_index)) * ...
                inverse_direction;
            t2 = (box_max(axis_index) - ray_origin(axis_index)) * ...
                inverse_direction;

            t_near = max(t_near, min(t1,t2));
            t_far  = min(t_far,  max(t1,t2));

            if t_near > t_far + tolerance
                hit = false;
                return;
            end
        end
    end

    hit = t_far > tolerance && t_near < maximum_distance;
end


function intersects = rayIntersectsTriangles( ...
    ray_origin, ray_direction, maximum_distance, ...
    v0, edge1, edge2, tolerance)
%RAYINTERSECTSTRIANGLES Vectorized two-sided Moller-Trumbore test.

    number_triangles = size(v0,1);
    ray_matrix = repmat(ray_direction, number_triangles, 1);

    h = cross(ray_matrix, edge2, 2);
    determinant = sum(edge1 .* h, 2);

    nonparallel = abs(determinant) > tolerance;

    if ~any(nonparallel)
        intersects = false;
        return;
    end

    inverse_determinant = zeros(number_triangles,1);
    inverse_determinant(nonparallel) = 1 ./ determinant(nonparallel);

    relative_origin = ray_origin - v0;

    barycentric_u = inverse_determinant .* ...
        sum(relative_origin .* h, 2);

    q = cross(relative_origin, edge1, 2);

    barycentric_v = inverse_determinant .* ...
        sum(ray_matrix .* q, 2);

    hit_distance = inverse_determinant .* sum(edge2 .* q, 2);

    valid_hit = ...
        nonparallel & ...
        barycentric_u >= -tolerance & ...
        barycentric_v >= -tolerance & ...
        barycentric_u + barycentric_v <= 1 + tolerance & ...
        hit_distance > tolerance & ...
        hit_distance < maximum_distance;

    intersects = any(valid_hit);
end
