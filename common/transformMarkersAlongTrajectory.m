function markers_world = transformMarkersAlongTrajectory( ...
    trajectory_positions, trajectory_rotations, markers_body)
%TRANSFORMMARKERSALONGTRAJECTORY Transform body-frame markers to world frame.
%
% Inputs:
%   trajectory_positions : K x 3 robot positions in world frame
%   trajectory_rotations : 3 x 3 x K body-to-world rotations
%   markers_body         : M x 3 marker positions in body frame
%
% Output:
%   markers_world        : K x M x 3 marker positions in world frame
%
% Transformation:
%   marker_world = position_world + R_body_to_world * marker_body

    K = size(trajectory_positions, 1);
    M = size(markers_body, 1);

    if size(trajectory_positions,2) ~= 3
        error('trajectory_positions must have size K x 3.');
    end

    if ~isequal(size(trajectory_rotations), [3, 3, K])
        error('trajectory_rotations must have size 3 x 3 x K.');
    end

    if size(markers_body,2) ~= 3
        error('markers_body must have size M x 3.');
    end

    markers_world = zeros(K, M, 3);

    for k = 1:K
        R = trajectory_rotations(:,:,k);
        position = trajectory_positions(k,:);

        markers_at_k = (R * markers_body')' + position;

        markers_world(k,:,:) = markers_at_k;
    end
end