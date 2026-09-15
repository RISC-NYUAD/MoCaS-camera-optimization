function score = cameraLayoutObjective(x, voxels, num_cams, k_req, roi_bounds, workspace_bounds)
%CAMERALAYOUTOBJECTIVE PSO objective with coverage + spacing + stereo quality.
%
%   x         : [1 x 5*num_cams]  -> [x,y,z,yaw,pitch] per camera
%   voxels    : [K x 3] sample points in ROI
%   num_cams  : number of cameras
%   k_req     : required # of cameras per voxel
%   roi_bounds: [3x2] bounds of the ROI (for stereo center)
%   workspace_bounds: [3x2] bounds of the full workspace (for inward constraint)

    % ---- reshape decision vector ----
    X = reshape(x, [5, num_cams])';  % [num_cams x 5]
    cam_pos   = X(:,1:3);
    cam_yaw   = X(:,4);
    cam_pitch = X(:,5);

    % Workspace center (for inward-looking constraint)
    centerW = [ mean(workspace_bounds(1,:)), ...
                mean(workspace_bounds(2,:)), ...
                mean(workspace_bounds(3,:)) ];

    % ==============================================================
    % 1) k-COVERAGE TERM
    % ==============================================================

    % Compute coverage and per-voxel visible counts
    [J, visible_counts] = evaluateCoverageOnCube(cam_pos, cam_yaw, cam_pitch, voxels, k_req);
    % J in [0,1] = fraction of voxels with >= k_req cameras

    % "Requirement": we want at least alpha fraction k-covered
    alpha_req = 1.0;  % 95% of ROI should be k-covered (tune this)
    P_cov = max(0, alpha_req - J).^2;

    % ==============================================================
    % 2) INTER-CAMERA DISTANCE PENALTY
    % ==============================================================

    d_min_cam = 4.0;   % minimum allowed distance between cameras (in your units)
    P_dist = 0;
    for i = 1:num_cams
        for j = i+1:num_cams
            d_ij = norm(cam_pos(i,:) - cam_pos(j,:));
            viol = max(0, d_min_cam - d_ij);
            P_dist = P_dist + viol^2;
        end
    end

    % ==============================================================
    % 3) BASELINE / STEREO QUALITY PENALTY
    % ==============================================================

    % Use ROI center as reference point
    roi_center = [ mean(roi_bounds(1,:)), ...
                   mean(roi_bounds(2,:)), ...
                   mean(roi_bounds(3,:)) ];

    % Rays from cameras to ROI center
    rays = zeros(num_cams,3);
    for i = 1:num_cams
        v = roi_center - cam_pos(i,:);
        if norm(v) < 1e-6
            % avoid zero-length vector
            rays(i,:) = [1 0 0];
        else
            rays(i,:) = v / norm(v);
        end
    end

    % desired angle range [alpha_min, alpha_max] in radians
    alpha_min = deg2rad(10);    % e.g. 10 degrees
    alpha_max = deg2rad(300);   % > pi → effectively no upper penalty

    P_stereo = 0;
    for i = 1:num_cams
        for j = i+1:num_cams
            dot_ij = dot(rays(i,:), rays(j,:));
            dot_ij = max(min(dot_ij,1),-1);  % clamp numerical noise
            alpha_ij = acos(dot_ij);

            viol_low  = max(0, alpha_min - alpha_ij);
            viol_high = max(0, alpha_ij - alpha_max);

            P_stereo = P_stereo + viol_low^2 + viol_high^2;
        end
    end

    % ==============================================================
    % 4) INWARD-LOOKING PENALTY (camera axis vs workspace center)
    % ==============================================================

    % Camera viewing directions from yaw/pitch (ZYX, no roll)
    dirs = zeros(num_cams,3);
    for i = 1:num_cams
        dirs(i,:) = [cos(cam_pitch(i))*cos(cam_yaw(i)), ...
                     cos(cam_pitch(i))*sin(cam_yaw(i)), ...
                     sin(cam_pitch(i))];
        dnorm = norm(dirs(i,:));
        if dnorm > 1e-6
            dirs(i,:) = dirs(i,:) / dnorm;
        else
            dirs(i,:) = [1 0 0];  % fallback
        end
    end

    % Penalize cameras that do not look towards the workspace center
    P_center = 0;
    cos_min = 0.0;   % >= 0 → center must be in front half-space
    % you can try cos_min = 0.3 or 0.5 later for stricter inward pointing

    for i = 1:num_cams
        v = centerW - cam_pos(i,:);
        vnorm = norm(v);
        if vnorm < 1e-6
            continue; % camera at center, ignore
        end
        v = v / vnorm;

        cos_beta = dot(dirs(i,:), v);   % alignment with workspace center

        viol = max(0, cos_min - cos_beta);
        P_center = P_center + viol^2;
    end

    % ==============================================================
    % 5) COMBINE TERMS INTO SINGLE SCORE
    % ==============================================================

    % We want to maximize score, PSO will minimize -score.
    % Tune these weights depending on how strict you want each term.
    lambda_cov    = 10.0;    % enforce coverage strongly
    lambda_dist   = 1.0;     % spacing importance
    lambda_stereo = 1.0;     % stereo geometry importance
    lambda_center = 1.0;     % inward-looking importance

    score = J ...
          - lambda_cov    * P_cov ...
          - lambda_dist   * P_dist ...
          - lambda_stereo * P_stereo ...
          - lambda_center * P_center;
end
