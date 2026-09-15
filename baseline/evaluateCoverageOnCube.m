function [J, visible_counts] = evaluateCoverageOnCube(cam_pos, cam_yaw, cam_pitch, voxels, k_req)
%EVALUATECOVERAGEONCUBE
%   J              : normalized soft k-coverage score in [0,1]
%   visible_counts : number of cameras seeing each voxel

    num_cams = size(cam_pos,1);
    K        = size(voxels,1);

    visible_counts = zeros(K,1);

    % --- FoV + range parameters ---
    fov_deg   = 75;
    fov_rad   = deg2rad(fov_deg);
    max_range = 10;
    min_range = 0.1;

    for i = 1:num_cams
        % direction from yaw,pitch
        dir = [cos(cam_pitch(i))*cos(cam_yaw(i)), ...
               cos(cam_pitch(i))*sin(cam_yaw(i)), ...
               sin(cam_pitch(i))];
        dir = dir / norm(dir);

        rel  = voxels - cam_pos(i,:);
        dist = vecnorm(rel,2,2);

        cos_angle = sum(rel .* dir, 2) ./ (dist + eps);
        in_fov    = cos_angle > cos(fov_rad/2);
        in_range  = dist > min_range & dist < max_range;

        visible_counts = visible_counts + (in_fov & in_range);
    end

    % --- SOFT COVERAGE: partial credit if g < k_req ---
    % c_k = min(1, g_k / k_req), then average over voxels
    soft_cov = min(1, visible_counts / k_req);
    J = mean(soft_cov);
end
