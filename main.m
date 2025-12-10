% User_Inputs
workspace_bounds = [0 5;
                    0 10;
                    0 5];

roi1_bounds = [0.5 1.5;
               1.0 2.0;
               0.5 1.5];

roi2_bounds = [3.5 4.5;
               7.5 8.5;
               1.0 2.0];

roi3_bounds = [2.0 3.0;
               4.0 5.0;
               3.5 4.5];
roi_bounds = [0.5 3.5;
               0.5 5.0;
               1.0 4.0];
%roi_bounds_all = cat(3, roi1_bounds, roi2_bounds, roi3_bounds);
%roi_bounds_all = workspace_bounds;
roi_bounds_all = cat(3, roi_bounds);

num_cams = 10;
k_req    = 8;


%%
t_start = tic;
[best_layout, voxels, initial_layout, output] = optimizeCameraLayoutCube(workspace_bounds, roi_bounds_all, num_cams, k_req);
total_time_sec = toc(t_start);

%%
% --- visualize layout with ALL regions and ALL voxels ---
% Update plot function to take roi_bounds_all and (optionally) voxels
plotCameraLayoutCube(best_layout, workspace_bounds, roi_bounds_all, voxels);

% --- extract camera params from best_layout ---
cam_pos   = best_layout(:,1:3);
cam_yaw   = best_layout(:,4);
cam_pitch = best_layout(:,5);

% --- recompute coverage on ALL ROI voxels (stacked from all regions) ---
[J, visible_counts_opt] = evaluateCoverageOnCube( ...
    cam_pos, cam_yaw, cam_pitch, voxels, k_req);

[J_init, visible_counts_init] = evaluateCoverageOnCube( ...
    initial_layout(:,1:3), initial_layout(:,4), initial_layout(:,5), voxels, k_req);

% --- print main result ---
num_voxels = numel(visible_counts_opt);
num_k      = sum(visible_counts_opt >= k_req);    % hard k-coverage count

fprintf('\n===== MULTI-ROI k-COVERAGE REPORT =====\n');
fprintf('Required k               : %d\n', k_req);
fprintf('Total ROI voxels (all)   : %d\n', num_voxels);
fprintf('Hard k-covered voxels    : %d\n', num_k);
fprintf('Hard k-coverage fraction : %.2f %%\n', 100 * num_k / num_voxels);
fprintf('Soft coverage J          : %.3f\n', J);   % from soft metric
fprintf('Total time        : %.2f s (%.2f min)\n', ...
            total_time_sec, total_time_sec/60);
fprintf('Iterations        : %d\n', output.iterations);
fprintf('=======================================\n');

% --- optional: print full view-count distribution ---
max_views = max(visible_counts_opt);
fprintf('\nView-count distribution (all regions combined):\n');
for v = 1:max_views
    fprintf('  Voxels seen by %d cameras: %d\n', ...
            v, sum(visible_counts_opt == v));
end

%%
% ---- Compute integer bins ----
vmin = 0;
vmax = max([10; visible_counts_init(:); visible_counts_opt(:)]);
bins = vmin:vmax;

% ---- Count frequencies for each layout ----
counts_init = histcounts(visible_counts_init, [bins-0.5, bins(end)+0.5]);
counts_opt  = histcounts(visible_counts_opt,  [bins-0.5, bins(end)+0.5]);

% ---- Convert to percentages ----
counts_init = counts_init / sum(counts_init) * 100;
counts_opt  = counts_opt  / sum(counts_opt)  * 100;

% ---- Grouped bar chart ----
figure('Color','w'); hold on; box on; grid on;

h = bar(bins, [counts_init; counts_opt]', 'grouped');
h(1).FaceColor = [0.65 0.65 0.65];   % gray
h(2).FaceColor = [0.30 0.50 0.90];   % blue

% ---- Make the columns wider ----
set(h, 'BarWidth', 1);   % <--- wider, 1.0 = max before touching

% ---- Labels ----
xlabel('Number of Cameras Observing Each Voxel', 'FontSize', 12);
ylabel('Percentage of Voxels (%)', 'FontSize', 12);
title(sprintf('Voxel Visibility Comparison (k_{req} = %d)', k_req), 'FontSize', 13);

xticks(bins);

% ---- Vertical k_req line ----
xline(k_req, '--r', 'LineWidth', 1.4, ...
    'Label', sprintf('k_{req} = %d', k_req), ...
    'LabelHorizontalAlignment','left', 'FontSize', 11);

legend({'Initial Layout', 'Optimized Layout'}, 'Location', 'northwest');
set(gca, 'FontSize', 11);
