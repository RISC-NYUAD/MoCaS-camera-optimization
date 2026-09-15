clc;
clear;
close all;

%% User settings

common_folder = ...
    'C:\Users\ama10362\Desktop\NYU\Vicon markers paper\codes\icara2027\common';

% This file provides the prescribed trajectory and its optimized layout.
trajectory_results_name = 'hellicalhpsuccess.mat';
%trajectory_results_name = 'Oscillatinghpcsuccess.mat';
%trajectory_results_name = 'optimizedline65to65m4hpc.mat';

% Replace this with the MAT-file saved by the old volumetric optimization.
base_results_name = 'base2.mat';

% Set false only when a comparison without robot self-occlusion is wanted.
use_self_occlusion = true;

marker_neighborhood = 0.025; % m

new_folder     = ...
    'C:\Users\ama10362\Desktop\NYU\Vicon markers paper\codes\icara2027\new';

addpath(new_folder);

%% Paths and input files

if ~isfolder(common_folder)
    error('Common folder was not found:\n%s', common_folder);
end

addpath(common_folder);

saved_folder = fullfile(common_folder, 'saved_layouts');

trajectory_results_file = fullfile( ...
    saved_folder, trajectory_results_name);

base_results_file = fullfile( ...
    saved_folder, base_results_name);

if ~isfile(trajectory_results_file)
    error('Trajectory-results file was not found:\n%s', ...
        trajectory_results_file);
end

if ~isfile(base_results_file)
    error(['Base-layout results file was not found:\n%s\n\n', ...
           'Set base_results_name to the filename produced by the ', ...
           'old volumetric optimization.'], ...
          base_results_file);
end

%% Load the two saved runs

trajectory_data = load(trajectory_results_file, 'saved_run');
base_data       = load(base_results_file, 'saved_run');

if ~isfield(trajectory_data, 'saved_run')
    error('The trajectory MAT-file does not contain saved_run.');
end

if ~isfield(base_data, 'saved_run')
    error('The base-layout MAT-file does not contain saved_run.');
end

trajectory_run = trajectory_data.saved_run;
base_run       = base_data.saved_run;

required_trajectory_fields = { ...
    'best_layout', 'plot_problem', 'num_cams', 'k_req', 'M_req'};

for field_index = 1:numel(required_trajectory_fields)
    field_name = required_trajectory_fields{field_index};

    if ~isfield(trajectory_run, field_name)
        error('The trajectory saved_run is missing field "%s".', ...
            field_name);
    end
end

if ~isfield(base_run, 'best_layout')
    error('The base-layout saved_run is missing best_layout.');
end

trajectory_layout = trajectory_run.best_layout;
base_layout       = base_run.best_layout;

if size(trajectory_layout,2) ~= 5 || size(base_layout,2) ~= 5
    error(['Both layouts must be N-by-5 matrices containing ', ...
           '[x, y, z, yaw, pitch].']);
end

fprintf('\nTrajectory results loaded from:\n%s\n', ...
    trajectory_results_file);
fprintf('\nBase layout loaded from:\n%s\n', ...
    base_results_file);

%% Reconstruct the complete trajectory problem

problem = trajectory_run.plot_problem;

required_problem_fields = { ...
    'workspace_bounds', 'camera', 'trajectory', 'markers_world'};

for field_index = 1:numel(required_problem_fields)
    field_name = required_problem_fields{field_index};

    if ~isfield(problem, field_name)
        error('The saved plot_problem is missing field "%s".', ...
            field_name);
    end
end

problem.num_cams = trajectory_run.num_cams;
problem.k_req    = trajectory_run.k_req;
problem.M_req    = trajectory_run.M_req;

if size(trajectory_layout,1) ~= problem.num_cams
    error(['The trajectory layout contains %d cameras, but the ', ...
           'saved problem requires %d cameras.'], ...
          size(trajectory_layout,1), problem.num_cams);
end

if size(base_layout,1) ~= problem.num_cams
    error(['The base layout contains %d cameras, but the ', ...
           'saved problem requires %d cameras.'], ...
          size(base_layout,1), problem.num_cams);
end

%% Load the body-frame marker configuration

marker_file = fullfile(common_folder, 'markers.mat');

if ~isfile(marker_file)
    error('Marker file was not found:\n%s', marker_file);
end

marker_data = load(marker_file, 'markers_body');

if ~isfield(marker_data, 'markers_body')
    error('markers.mat does not contain markers_body.');
end

problem.markers_body = marker_data.markers_body;

if size(problem.markers_world,2) ~= size(problem.markers_body,1)
    error(['The saved trajectory contains %d markers, whereas ', ...
           'markers.mat contains %d markers.'], ...
          size(problem.markers_world,2), ...
          size(problem.markers_body,1));
end

%% Prepare the robot mesh when self-occlusion is enabled

problem.visibility.marker_neighborhood = marker_neighborhood;
problem.visibility.use_self_occlusion   = use_self_occlusion;

if use_self_occlusion
    stl_file = fullfile(common_folder, 'omni.STL');

    if ~isfile(stl_file)
        error('Robot STL file was not found:\n%s', stl_file);
    end

    raw_mesh = stlread(stl_file);

    mesh_vertices = double(raw_mesh.Points) / 1000;
    mesh_faces    = double(raw_mesh.ConnectivityList);

    problem.robot_mesh = triangulation( ...
        mesh_faces, mesh_vertices);

    problem.occlusion_mesh = ...
        prepareOcclusionMesh(problem.robot_mesh);
end

%% Evaluate the old base layout on the prescribed trajectory

base_evaluation_start = tic;

[visibility_base, visible_counts_base] = ...
    evaluateTrajectoryVisibility(base_layout, problem);

[trackable_base, valid_markers_base, marker_validity_base] = ...
    evaluateTrackability( ...
        visible_counts_base, ...
        problem.k_req, ...
        problem.M_req);

base_evaluation_time = toc(base_evaluation_start);

%% Reevaluate the trajectory-optimized layout under identical conditions

trajectory_evaluation_start = tic;

[visibility_trajectory, visible_counts_trajectory] = ...
    evaluateTrajectoryVisibility(trajectory_layout, problem);

[trackable_trajectory, valid_markers_trajectory, ...
        marker_validity_trajectory] = ...
    evaluateTrackability( ...
        visible_counts_trajectory, ...
        problem.k_req, ...
        problem.M_req);

trajectory_evaluation_time = toc(trajectory_evaluation_start);

%% Camera-layout values

fprintf('\n===== BASE VOLUMETRIC LAYOUT =====\n');
disp(base_layout);

fprintf('\n===== TRAJECTORY-OPTIMIZED LAYOUT =====\n');
disp(trajectory_layout);

%% Tracking comparison

t = problem.trajectory.t(:);
K = numel(t);

failure_samples_base       = find(~trackable_base);
failure_samples_trajectory = find(~trackable_trajectory);

if use_self_occlusion
    occlusion_description = 'enabled';
else
    occlusion_description = 'disabled';
end

fprintf('\n===== LAYOUT COMPARISON =====\n');
fprintf('Number of cameras             : %d\n', problem.num_cams);
fprintf('Required views per marker     : %d\n', problem.k_req);
fprintf('Required valid markers        : %d\n', problem.M_req);
fprintf('Trajectory samples            : %d\n', K);
fprintf('Robot self-occlusion          : %s\n', ...
    occlusion_description);

fprintf('\nBase volumetric layout:\n');
fprintf('Trackable samples             : %d/%d\n', ...
    sum(trackable_base), K);
fprintf('Tracking percentage           : %.2f %%\n', ...
    100 * mean(trackable_base));
fprintf('Minimum valid markers         : %d\n', ...
    min(valid_markers_base));
fprintf('Mean valid markers            : %.2f\n', ...
    mean(valid_markers_base));
fprintf('Failure samples               : %d\n', ...
    numel(failure_samples_base));
fprintf('Evaluation time               : %.3f s\n', ...
    base_evaluation_time);

fprintf('\nTrajectory-optimized layout:\n');
fprintf('Trackable samples             : %d/%d\n', ...
    sum(trackable_trajectory), K);
fprintf('Tracking percentage           : %.2f %%\n', ...
    100 * mean(trackable_trajectory));
fprintf('Minimum valid markers         : %d\n', ...
    min(valid_markers_trajectory));
fprintf('Mean valid markers            : %.2f\n', ...
    mean(valid_markers_trajectory));
fprintf('Failure samples               : %d\n', ...
    numel(failure_samples_trajectory));
fprintf('Evaluation time               : %.3f s\n', ...
    trajectory_evaluation_time);
fprintf('=============================\n');

%% Display failure times

if isempty(failure_samples_base)
    fprintf('\nThe base volumetric layout has no tracking failures.\n');
else
    fprintf('\nBase-layout failure times (s):\n');
    disp(t(failure_samples_base).');
end

if isempty(failure_samples_trajectory)
    fprintf(['The trajectory-optimized layout has no ', ...
             'tracking failures.\n']);
else
    fprintf('\nTrajectory-layout failure times (s):\n');
    disp(t(failure_samples_trajectory).');
end

%% Plot valid markers over time

figure( ...
    'Color', 'w', ...
    'Name', 'Layout Tracking Comparison');

hold on;
grid on;
box on;

plot(t, valid_markers_base, ...
    '--', ...
    'Color', [0.45 0.45 0.45], ...
    'LineWidth', 1.8);

plot(t, valid_markers_trajectory, ...
    '-', ...
    'Color', [0.20 0.45 0.85], ...
    'LineWidth', 1.8);

yline(problem.M_req, '--r', ...
    'LineWidth', 1.5, ...
    'Label', 'M_{req}', ...
    'LabelHorizontalAlignment', 'left');

xlabel('Time (s)');
ylabel('Number of Valid Markers');
title('Trackability Along the Prescribed Trajectory');

legend( ...
    {'Base volumetric layout', ...
     'Trajectory-optimized layout', ...
     'Tracking requirement'}, ...
    'Location', 'best');

ylim([0, size(problem.markers_world,2) + 0.5]);
set(gca, 'FontSize', 12);

%% Plot binary tracking status

figure( ...
    'Color', 'w', ...
    'Name', 'Rigid-Body Tracking Status Comparison');

hold on;
grid on;
box on;

stairs(t, double(trackable_base), ...
    '--', ...
    'Color', [0.45 0.45 0.45], ...
    'LineWidth', 1.8);

stairs(t, double(trackable_trajectory), ...
    '-', ...
    'Color', [0.20 0.45 0.85], ...
    'LineWidth', 1.8);

xlabel('Time (s)');
ylabel('Tracking Status');
title('Rigid-Body Tracking Status');

yticks([0 1]);
yticklabels({'Not trackable', 'Trackable'});
ylim([-0.1 1.1]);

legend( ...
    {'Base volumetric layout', ...
     'Trajectory-optimized layout'}, ...
    'Location', 'best');

set(gca, 'FontSize', 12);

%% Plot both layouts on the same prescribed trajectory

plotTrajectoryCameraLayout( ...
    base_layout, ...
    problem, ...
    trackable_base, ...
    'Base Volumetric Layout on Prescribed Trajectory');

plotTrajectoryCameraLayout( ...
    trajectory_layout, ...
    problem, ...
    trackable_trajectory, ...
    'Trajectory-Optimized Camera Layout');

%% Optional variables retained in the workspace for further analysis

comparison.base.layout             = base_layout;
comparison.base.visibility         = visibility_base;
comparison.base.visible_counts     = visible_counts_base;
comparison.base.marker_validity    = marker_validity_base;
comparison.base.valid_markers      = valid_markers_base;
comparison.base.trackable          = trackable_base;

comparison.trajectory.layout          = trajectory_layout;
comparison.trajectory.visibility      = visibility_trajectory;
comparison.trajectory.visible_counts  = visible_counts_trajectory;
comparison.trajectory.marker_validity = marker_validity_trajectory;
comparison.trajectory.valid_markers   = valid_markers_trajectory;
comparison.trajectory.trackable       = trackable_trajectory;

comparison.problem = problem;

