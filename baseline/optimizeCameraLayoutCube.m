function [best_layout, voxels, initial_layout,output] = optimizeCameraLayoutCube(workspace_bounds, roi_bounds_all, num_cams, k_req)
%OPTIMIZECAMERALAYOUTCUBE Optimize camera poses for k-coverage in an ROI cube.
%
%   workspace_bounds : [3x2] [xmin xmax; ymin ymax; zmin zmax]
%                      physical workspace where cameras are allowed
%   roi_bounds       : [3x2] smaller cube of interest inside workspace
%   num_cams         : number of cameras
%   k_req            : required number of cameras per voxel in ROI

    M = size(roi_bounds_all, 3);

    % ---- 1) Assert all ROIs lie inside the workspace ----
    for m = 1:M
        roi_bounds_m = roi_bounds_all(:,:,m);
        assertROIInsideWorkspace(workspace_bounds, roi_bounds_m);
    end

    % ---- 2) Discretize ALL regions into voxels and stack ----
    nx = 10; ny = 10; nz = 10;   % can refine later
    voxels = [];

    for m = 1:M
        rb = roi_bounds_all(:,:,m);

        [X, Y, Z] = ndgrid( ...
            linspace(rb(1,1), rb(1,2), nx), ...
            linspace(rb(2,1), rb(2,2), ny), ...
            linspace(rb(3,1), rb(3,2), nz));

        voxels_m = [X(:), Y(:), Z(:)];
        voxels   = [voxels; voxels_m];   % stack all regions
    end

     % ---- 3) Global ROI bounds (for stereo center / plots, etc.) ----
    x_min = min(roi_bounds_all(1,1,:), [], 'all');
    x_max = max(roi_bounds_all(1,2,:), [], 'all');
    y_min = min(roi_bounds_all(2,1,:), [], 'all');
    y_max = max(roi_bounds_all(2,2,:), [], 'all');
    z_min = min(roi_bounds_all(3,1,:), [], 'all');
    z_max = max(roi_bounds_all(3,2,:), [], 'all');

    roi_global_bounds = [x_min x_max;
                         y_min y_max;
                         z_min z_max];
    % --- decision variables: [x y z yaw pitch] per camera (roll fixed) ---
    nVars = num_cams * 5;
    x0 = initCamerasOnWorkspaceTop_lookBase(workspace_bounds, num_cams);
    initial_layout = reshape(x0, [5, num_cams])';

    SwarmSize = 3000;
    initSwarm = zeros(SwarmSize, nVars);
    
    initSwarm(1,:) = x0;
    noise_scale = 0.05;
    
    for s = 2:SwarmSize
        initSwarm(s,:) = x0 + noise_scale * randn(1, nVars);
    end

    % --- hard bounds: cameras live inside the workspace (not ROI) ---
    pos_lb = workspace_bounds(:,1)';   % [xmin_w, ymin_w, zmin_w]
    pos_ub = workspace_bounds(:,2)';   % [xmax_w, ymax_w, zmax_w]

    yaw_lb   = -pi * ones(1, num_cams);
    yaw_ub   =  pi * ones(1, num_cams);
    
    pitch_lb = -pi/3 * ones(1, num_cams);
    pitch_ub =  pi/3 * ones(1, num_cams);

    lb = [];
    ub = [];
    for i = 1:num_cams
        lb = [lb, pos_lb, yaw_lb(i),   pitch_lb(i)];
        ub = [ub, pos_ub, yaw_ub(i),   pitch_ub(i)];
    end

    %% --- PSO options ---
    options = optimoptions('particleswarm', ...
        'SwarmSize', SwarmSize, ...
        'MaxIterations', 1000, ...
        'Display', 'iter', ...
        'InitialSwarmMatrix', initSwarm);

    % --- objective: PSO minimizes, so use negative coverage ---
    objectiveFcn = @(x) -cameraLayoutObjective(x, voxels, num_cams, k_req, roi_global_bounds, workspace_bounds);

    % --- run PSO ---
    [z_opt, fval, exitflag, output] = particleswarm(objectiveFcn, nVars, lb, ub, options);
    
    % --- reshape result: [num_cams x 5] [x y z yaw pitch] ---
    best_layout = reshape(z_opt, [5, num_cams])';

%% ---- fmincon objective (minimize negative score) ----
% fminObj = @(x) -cameraLayoutObjective(x, voxels, num_cams, k_req, roi_global_bounds, workspace_bounds);
% 
%    %---- fmincon options ----
% opts = optimoptions('fmincon', ...
%   'Algorithm','sqp', ...
%   'Display','iter', ...
%   'MaxIterations',1000, ...
%   'MaxFunctionEvaluations',2e5, ...
%   'FiniteDifferenceType','forward', ...
%   'FiniteDifferenceStepSize', 1e-2, ...   % <-- key change (try 1e-2 to 1e-1)
%   'OptimalityTolerance',1e-6);
% 
% 
% % No nonlinear constraints in your current formulation
% A = []; b = []; Aeq = []; beq = []; nonlcon = [];
% 
% [x_fmin, fval_fmin, exitflag_fmin, output_fmin] = fmincon( ...
%     fminObj, x0, A, b, Aeq, beq, lb, ub, nonlcon, opts);
% 
% best_layout_fmin = reshape(x_fmin, [5, num_cams])';
% score_fmin = -fval_fmin;  % convert back to "maximize" score
% 
% fprintf('fmincon done. score = %.4f, exitflag = %d, iters = %d\n', ...
%     score_fmin, exitflag_fmin, output_fmin.iterations);
% 
% best_layout = best_layout_fmin;
% output = output_fmin;   % or output_fmin with extra fields

end
function assertROIInsideWorkspace(workspace_bounds, roi_bounds)
%ASSERTROIINSIDEWORKSPACE throws error if ROI is not fully inside workspace.

    for dim = 1:3
        if roi_bounds(dim,1) < workspace_bounds(dim,1) || ...
           roi_bounds(dim,2) > workspace_bounds(dim,2)
            error('ROI in dimension %d is not contained in the workspace bounds.', dim);
        end
    end
end
% 
% function x0 = initCamerasOnWorkspaceTop_lookBase(workspace_bounds, num_cams)
%     % workspace_bounds: [3 x 2]
%     %   [xWmin xWmax;
%     %    yWmin yWmax;
%     %    zWmin zWmax]
%     %
%     % Returns x0: 1 x (5*num_cams)  [x y z yaw pitch] per camera
% 
%     % Workspace center in XY
%     cxW = mean(workspace_bounds(1,:));
%     cyW = mean(workspace_bounds(2,:));
% 
%     % Base center (what we look at)
%     base_center = [cxW, cyW, workspace_bounds(3,1)];  % zWmin
% 
%     % Top surface of workspace (where cameras sit)
%     top_z = workspace_bounds(3,2);  % zWmax
% 
%     % Workspace XY size
%     dxW = diff(workspace_bounds(1,:));
%     dyW = diff(workspace_bounds(2,:));
% 
%     % Radius: stay inside workspace top surface
%     max_radius = 0.45 * min(dxW, dyW);   % 45% of min side
% 
%     % Angular positions around the ring
%     angles = linspace(0, 2*pi, num_cams+1);
%     angles(end) = [];  % remove duplicate
% 
%     cam_params = zeros(num_cams, 5);
% 
%     for i = 1:num_cams
%         th = angles(i);
% 
%         % Camera position on top surface of workspace
%         x = cxW + max_radius * cos(th);
%         y = cyW + max_radius * sin(th);
%         z = top_z;
% 
%         % Direction vector towards center of workspace base
%         v = base_center - [x, y, z];
%         r_xy  = norm(v(1:2));
%         yaw   = atan2(v(2), v(1));   % in XY plane
%         pitch = atan2(v(3), r_xy);   % elevation
% 
%         cam_params(i,:) = [x, y, z, yaw, pitch];
%     end
% 
%     % Flatten to decision vector [1 x (5*num_cams)]
%     x0 = reshape(cam_params.', 1, []);
% end

function x0 = initCamerasOnWorkspaceTop_lookBase(workspace_bounds, num_cams)
    % workspace_bounds: [3 x 2]
    %   [xWmin xWmax;
    %    yWmin yWmax;
    %    zWmin zWmax]
    %
    % Returns x0: 1 x (5*num_cams)  [x y z yaw pitch] per camera
    %
    % Cameras are placed on the top surface (z = zWmax),
    % evenly spaced along the perimeter of the top rectangle,
    % each one oriented toward the geometric center of the workspace.

    xWmin = workspace_bounds(1,1);
    xWmax = workspace_bounds(1,2);
    yWmin = workspace_bounds(2,1);
    yWmax = workspace_bounds(2,2);
    zWmin = workspace_bounds(3,1);
    zWmax = workspace_bounds(3,2);

    % Geometric center of the full workspace (target point)
    cxW = 0.5 * (xWmin + xWmax);
    cyW = 0.5 * (yWmin + yWmax);
    czW = 0.5 * (zWmin + zWmax);
    workspace_center = [cxW, cyW, czW];

    % Top surface z coordinate
    top_z = 0.8*zWmax;

    % Perimeter lengths
    dxW = xWmax - xWmin;
    dyW = yWmax - yWmin;
    perim = 2*(dxW + dyW);

    cam_params = zeros(num_cams, 5);

    % Evenly spaced along perimeter [0, perim)
    for i = 1:num_cams
        s = (i-1) / num_cams * perim;  % arc-length parameter

        if s <= dxW
            % bottom edge: (xWmin -> xWmax, y = yWmin)
            x = xWmin + s;
            y = yWmin;
        elseif s <= dxW + dyW
            % right edge: (x = xWmax, yWmin -> yWmax)
            x = xWmax;
            y = yWmin + (s - dxW);
        elseif s <= 2*dxW + dyW
            % top edge: (xWmax -> xWmin, y = yWmax)
            x = xWmax - (s - (dxW + dyW));
            y = yWmax;
        else
            % left edge: (x = xWmin, yWmax -> yWmin)
            x = xWmin;
            y = yWmax - (s - (2*dxW + dyW));
        end

        z = top_z;

        % Direction vector from camera to workspace center
        v    = workspace_center - [x, y, z];
        r_xy = norm(v(1:2));
        yaw   = atan2(v(2), v(1));   % azimuth in XY plane
        pitch = atan2(v(3), r_xy);   % elevation

        cam_params(i,:) = [x, y, z, yaw, pitch];
    end

    % Flatten to decision vector [1 x (5*num_cams)]
    x0 = reshape(cam_params.', 1, []);
end


