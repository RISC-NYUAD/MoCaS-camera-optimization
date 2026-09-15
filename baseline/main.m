% User_Inputs

workspace_bounds = [-8 8;
                    -8 8;
                     0 6];

% roi1_bounds = [0.5 1.5;
%                1.0 2.0;
%                0.5 1.5];
% 
% roi2_bounds = [3.5 4.5;
%                7.5 8.5;
%                1.0 2.0];
% 
% roi3_bounds = [2.0 3.0;
%                4.0 5.0;
%                3.5 4.5];
% roi_bounds = [0.5 3.5;
%                0.5 5.0;
%                1.0 4.0];
% roi_bounds_all = cat(3, roi1_bounds, roi2_bounds, roi3_bounds);
roi_bounds_all = workspace_bounds;
%roi_bounds_all = cat(3, roi_bounds);

num_cams = 8;
k_req    = 4;


%%
t_start = tic;
[best_layout, voxels, initial_layout, output] = optimizeCameraLayoutCube(workspace_bounds, roi_bounds_all, num_cams, k_req);
total_time_sec = toc(t_start);

%%
% --- visualize layout with ALL regions and ALL voxels ---
% Update plot function to take roi_bounds_all and (optionally) voxels

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

% --- blind spots (hard k-coverage failures) ---
blind_mask_opt  = (visible_counts_opt  < k_req);
blind_mask_init = (visible_counts_init < k_req);

blind_spots_opt  = voxels(blind_mask_opt,  :);
blind_spots_init = voxels(blind_mask_init, :);

% --- report counts ---
fprintf('\nBlind spots (init) : %d voxels (%.2f%%)\n', ...
    size(blind_spots_init,1), 100*mean(blind_mask_init));
fprintf('Blind spots (opt)  : %d voxels (%.2f%%)\n', ...
    size(blind_spots_opt,1), 100*mean(blind_mask_opt));

plotCameraLayoutCube(best_layout, workspace_bounds, roi_bounds_all, voxels, true, blind_spots_opt);
plotCameraLayoutCube(initial_layout, workspace_bounds, roi_bounds_all, voxels, true, blind_spots_init);


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

%% Save volumetric layout for later trajectory comparisons

% Use the same saved_layouts folder as the new trajectory code
results_folder = fullfile(common_folder, 'saved_layouts');

if ~isfolder(results_folder)
    mkdir(results_folder);
end

saved_run = struct();

%% Layouts

saved_run.best_layout    = best_layout;
saved_run.initial_layout = initial_layout;

%% Identify the source of this layout

saved_run.run_type = 'static_volumetric_optimization';
saved_run.description = ...
    ['Camera layout optimized for volumetric workspace coverage. ', ...
     'It can later be evaluated on prescribed trajectories.'];

%% Camera-layout information

saved_run.plot_problem.workspace_bounds = workspace_bounds;

saved_run.num_cams = num_cams;
saved_run.k_req    = k_req;

%% Original volumetric optimization data

saved_run.volumetric.roi_bounds_all = roi_bounds_all;
saved_run.volumetric.voxels         = voxels;

saved_run.volumetric.J_opt  = J;
saved_run.volumetric.J_init = J_init;

saved_run.volumetric.visible_counts_opt  = visible_counts_opt;
saved_run.volumetric.visible_counts_init = visible_counts_init;

saved_run.volumetric.blind_spots_opt  = blind_spots_opt;
saved_run.volumetric.blind_spots_init = blind_spots_init;

saved_run.volumetric.hard_coverage_opt = ...
    mean(visible_counts_opt >= k_req);

saved_run.volumetric.hard_coverage_init = ...
    mean(visible_counts_init >= k_req);

%% Optimization information

saved_run.output         = output;
saved_run.total_time_sec = total_time_sec;
saved_run.date_created   = datetime('now');

%% Save with an identifiable filename

timestamp = char(datetime('now', ...
    'Format', 'yyyyMMdd_HHmmss'));

results_file = fullfile( ...
    results_folder, ...
    ['volumetric_layout_' timestamp '.mat']);

save(results_file, 'saved_run');

fprintf('\nSaved volumetric camera layout to:\n%s\n', ...
    results_file);