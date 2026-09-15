function mesh_data = prepareOcclusionMesh(robot_mesh, leaf_size)
%PREPAREOCCLUSIONMESH Precompute triangle data and a BVH for ray casting.
%
% mesh_data = prepareOcclusionMesh(robot_mesh)
% mesh_data = prepareOcclusionMesh(robot_mesh, leaf_size)
%
% The STL is kept at full resolution. A bounding-volume hierarchy (BVH) is
% built once so that each camera-marker ray only tests triangles in nearby
% leaf nodes instead of scanning the complete mesh. A leaf size between 8
% and 16 is usually a good choice.

    if nargin < 2 || isempty(leaf_size)
        leaf_size = 12;
    end

    if ~isa(robot_mesh, 'triangulation')
        error('robot_mesh must be a triangulation object.');
    end

    if ~isscalar(leaf_size) || ~isfinite(leaf_size) || ...
            leaf_size < 1 || leaf_size ~= floor(leaf_size)
        error('leaf_size must be a positive integer.');
    end

    vertices = double(robot_mesh.Points);
    faces    = double(robot_mesh.ConnectivityList);

    number_triangles = size(faces,1);

    if number_triangles == 0
        error('robot_mesh does not contain any triangles.');
    end

    triangle_v0 = vertices(faces(:,1),:);
    triangle_v1 = vertices(faces(:,2),:);
    triangle_v2 = vertices(faces(:,3),:);

    triangle_edge1 = triangle_v1 - triangle_v0;
    triangle_edge2 = triangle_v2 - triangle_v0;

    triangle_min = min(min(triangle_v0, triangle_v1), triangle_v2);
    triangle_max = max(max(triangle_v0, triangle_v1), triangle_v2);
    triangle_centroid = (triangle_v0 + triangle_v1 + triangle_v2) / 3;

    %% Preallocate the BVH

    % A binary BVH has fewer than 2*T nodes for T triangles.
    maximum_nodes = max(1, 2 * number_triangles);

    node_min       = zeros(maximum_nodes, 3);
    node_max       = zeros(maximum_nodes, 3);
    left_child     = zeros(maximum_nodes, 1, 'uint32');
    right_child    = zeros(maximum_nodes, 1, 'uint32');
    triangle_start = zeros(maximum_nodes, 1, 'uint32');
    triangle_count = zeros(maximum_nodes, 1, 'uint32');

    ordered_triangles = zeros(number_triangles, 1, 'uint32');

    node_count       = 0;
    ordered_count    = 0;
    maximum_depth    = 0;

    root_node = buildNode((1:number_triangles)', 1);

    %% Reorder triangles so every leaf occupies one contiguous block

    triangle_order = double(ordered_triangles(1:ordered_count));

    mesh_data.v0           = triangle_v0(triangle_order,:);
    mesh_data.edge1        = triangle_edge1(triangle_order,:);
    mesh_data.edge2        = triangle_edge2(triangle_order,:);
    mesh_data.triangle_min = triangle_min(triangle_order,:);
    mesh_data.triangle_max = triangle_max(triangle_order,:);

    mesh_data.number_triangles = number_triangles;

    mesh_data.bvh.root           = root_node;
    mesh_data.bvh.node_min       = node_min(1:node_count,:);
    mesh_data.bvh.node_max       = node_max(1:node_count,:);
    mesh_data.bvh.left_child     = left_child(1:node_count);
    mesh_data.bvh.right_child    = right_child(1:node_count);
    mesh_data.bvh.triangle_start = triangle_start(1:node_count);
    mesh_data.bvh.triangle_count = triangle_count(1:node_count);
    mesh_data.bvh.max_depth      = maximum_depth;
    mesh_data.bvh.leaf_size      = leaf_size;

    number_leaves = nnz(mesh_data.bvh.triangle_count > 0);

    fprintf(['Prepared full occlusion mesh with %d triangles, ', ...
             '%d BVH nodes and %d leaves.\n'], ...
        number_triangles, node_count, number_leaves);


    function node_index = buildNode(triangle_ids, depth)
    %BUILDNODE Recursively split triangles along their widest centroid axis.

        node_count = node_count + 1;
        current_node = node_count;
        node_index = uint32(current_node);

        node_min(current_node,:) = min(triangle_min(triangle_ids,:), [], 1);
        node_max(current_node,:) = max(triangle_max(triangle_ids,:), [], 1);

        maximum_depth = max(maximum_depth, depth);

        if numel(triangle_ids) <= leaf_size
            first_triangle = ordered_count + 1;
            number_in_leaf = numel(triangle_ids);

            ordered_triangles( ...
                first_triangle:first_triangle + number_in_leaf - 1) = ...
                uint32(triangle_ids);

            triangle_start(current_node) = uint32(first_triangle);
            triangle_count(current_node) = uint32(number_in_leaf);
            ordered_count = ordered_count + number_in_leaf;
            return;
        end

        centroid_bounds_min = ...
            min(triangle_centroid(triangle_ids,:), [], 1);
        centroid_bounds_max = ...
            max(triangle_centroid(triangle_ids,:), [], 1);

        [~, split_axis] = max(centroid_bounds_max - centroid_bounds_min);

        [~, order] = sort( ...
            triangle_centroid(triangle_ids, split_axis), 'ascend');
        triangle_ids = triangle_ids(order);

        split_index = floor(numel(triangle_ids) / 2);

        left_child(current_node) = ...
            buildNode(triangle_ids(1:split_index), depth + 1);
        right_child(current_node) = ...
            buildNode(triangle_ids(split_index + 1:end), depth + 1);
    end
end
