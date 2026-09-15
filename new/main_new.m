clc;
clear;
close all;

%% Paths

common_folder = ...
    'C:\Users\ama10362\Desktop\NYU\Vicon markers paper\codes\icara2027\common';

if ~isfolder(common_folder)
    error('Common folder was not found: %s', common_folder);
end
addpath(common_folder);


%% Workspace

problem.workspace_bounds = [-8 8;
                            -8 8;
                            0 6];

%% Camera and tracking requirements

problem.num_cams = 8;   % Fixed initially
problem.k_req    = 4;    % Cameras required to observe each marker
problem.M_req    = 3;    % Valid markers required to track the rigid body

problem.camera.fov_deg   = 75;
problem.camera.min_range = 0.1;
problem.camera.max_range = 10;

problem.camera.yaw_bounds   = [-pi, pi];
problem.camera.pitch_bounds = [-pi/3, pi/3];

%% Generate prescribed trajectory

duration = 5;
dt       = 0.05;

% problem.trajectory = generateTrajectory(duration, dt);
problem.trajectory = ...
    generateHelicalDescentTrajectory(duration, dt);
% problem.trajectory = ...
%     generateForwardOscillatingTrajectory(duration, dt);
 visualizeTrajectory(problem.trajectory, problem.workspace_bounds);
%% Load marker configuration

marker_file = fullfile(common_folder, 'markers.mat');

% Create markers.mat automatically if it does not exist
if ~isfile(marker_file)

    markers_body_mm = [
       130   410   255
       340    40    20
       390  -160   165
       -65   395   -15
      -115  -155  -220
        75  -155     0
    ];

    % Convert millimetres to metres
    markers_body = markers_body_mm / 1000;

    save(marker_file, 'markers_body');
end

marker_data = load(marker_file, 'markers_body');
problem.markers_body = marker_data.markers_body;

%% Load and scale STL

stl_file = fullfile(common_folder, 'omni.STL');
raw_mesh = stlread(stl_file);

mesh_vertices = raw_mesh.Points / 1000;
mesh_faces    = raw_mesh.ConnectivityList;

problem.robot_mesh = triangulation( ...
    mesh_faces, mesh_vertices);

problem.occlusion_mesh = ...
    prepareOcclusionMesh(problem.robot_mesh);

problem.visibility.marker_neighborhood = 0.025;
visualizeMarkersOnSTL(problem.robot_mesh, problem.markers_body);

%% Compute marker positions along the trajectory

problem.markers_world = transformMarkersAlongTrajectory( ...
    problem.trajectory.p, ...
    problem.trajectory.R, ...
    problem.markers_body);

% fprintf('Marker trajectory size: %d samples x %d markers x %d coordinates\n', ...
%     size(problem.markers_world,1), ...
%     size(problem.markers_world,2), ...
%     size(problem.markers_world,3));

%% Optimize the camera layout

t_start = tic;

pool = parpool('local', 8);

if isempty(pool)
    pool = parpool('local');   % Uses configured number of workers
end

fprintf('Running PSO using %d parallel workers.\n', ...
    pool.NumWorkers);

problem.optimization.swarm_size     = 3000;%3000;
problem.optimization.max_iterations = 1000;%1000;
problem.optimization.random_seed    = 1;

problem.camera.min_spacing = 4;

problem.camera.alpha_min = deg2rad(10);
problem.camera.alpha_max = deg2rad(170);

problem.objective.beta          = 0.5;
problem.objective.lambda_track  = 100;
problem.objective.lambda_dist   = 0.1;
problem.objective.lambda_stereo = 0.1;
problem.objective.lambda_center = 0.1;
problem.visibility.use_self_occlusion = true;

[best_layout, initial_layout, output] = ...
    optimizeTrajectoryCameraLayout(problem);

total_time_sec = toc(t_start);


%% Evaluate layouts without self-occlusion

problem.visibility.use_self_occlusion = false;

[~, visible_counts_opt_no_occlusion] = ...
    evaluateTrajectoryVisibility(best_layout, problem);

[~, visible_counts_init_no_occlusion] = ...
    evaluateTrajectoryVisibility(initial_layout, problem);

[trackable_opt_no_occlusion, valid_markers_opt_no_occlusion] = ...
    evaluateTrackability( ...
        visible_counts_opt_no_occlusion, ...
        problem.k_req, ...
        problem.M_req);

[trackable_init_no_occlusion, valid_markers_init_no_occlusion] = ...
    evaluateTrackability( ...
        visible_counts_init_no_occlusion, ...
        problem.k_req, ...
        problem.M_req);

%% Evaluate layouts with self-occlusion

problem.visibility.use_self_occlusion = true;

occlusion_start = tic;

[visibility_opt, visible_counts_opt] = ...
    evaluateTrajectoryVisibility(best_layout, problem);

[visibility_init, visible_counts_init] = ...
    evaluateTrajectoryVisibility(initial_layout, problem);

occlusion_evaluation_time = toc(occlusion_start);

[trackable_opt, valid_markers_opt] = evaluateTrackability( ...
    visible_counts_opt, problem.k_req, problem.M_req);

[trackable_init, valid_markers_init] = evaluateTrackability( ...
    visible_counts_init, problem.k_req, problem.M_req);

%% Self-occlusion comparison

removed_observations_opt = sum( ...
    visible_counts_opt_no_occlusion - visible_counts_opt, 'all');

removed_observations_init = sum( ...
    visible_counts_init_no_occlusion - visible_counts_init, 'all');

fprintf('\n===== SELF-OCCLUSION REPORT =====\n');
fprintf('Evaluation time                 : %.3f s\n', ...
    occlusion_evaluation_time);

fprintf('\nInitial layout:\n');
fprintf('Tracking without occlusion      : %.2f %%\n', ...
    100 * mean(trackable_init_no_occlusion));
fprintf('Tracking with occlusion         : %.2f %%\n', ...
    100 * mean(trackable_init));
fprintf('Observations removed            : %d\n', ...
    removed_observations_init);

fprintf('\nOptimized layout:\n');
fprintf('Tracking without occlusion      : %.2f %%\n', ...
    100 * mean(trackable_opt_no_occlusion));
fprintf('Tracking with occlusion         : %.2f %%\n', ...
    100 * mean(trackable_opt));
fprintf('Observations removed            : %d\n', ...
    removed_observations_opt);
fprintf('=================================\n');

%% Results

K = numel(trackable_opt);

fprintf('\n===== TRAJECTORY TRACKING REPORT =====\n');
fprintf('Number of cameras              : %d\n', problem.num_cams);
fprintf('Required views per marker      : %d\n', problem.k_req);
fprintf('Required valid markers         : %d\n', problem.M_req);
fprintf('Trajectory samples             : %d\n', K);

fprintf('\nInitial layout:\n');
fprintf('Trackable samples              : %d\n', sum(trackable_init));
fprintf('Trajectory tracking percentage: %.2f %%\n', ...
    100 * mean(trackable_init));
fprintf('Minimum valid markers          : %d\n', ...
    min(valid_markers_init));

fprintf('\nOptimized layout:\n');
fprintf('Trackable samples              : %d\n', sum(trackable_opt));
fprintf('Trajectory tracking percentage: %.2f %%\n', ...
    100 * mean(trackable_opt));
fprintf('Minimum valid markers          : %d\n', ...
    min(valid_markers_opt));

fprintf('\nOptimization time              : %.2f s (%.2f min)\n', ...
    total_time_sec, total_time_sec/60);
fprintf('PSO iterations                 : %d\n', output.iterations);
fprintf('======================================\n');

%% Tracking failure samples

failure_samples_init = find(~trackable_init);
failure_samples_opt  = find(~trackable_opt);

fprintf('\nTracking failures:\n');
fprintf('Initial layout   : %d samples\n', numel(failure_samples_init));
fprintf('Optimized layout : %d samples\n', numel(failure_samples_opt));

%% Plot valid markers along the trajectory

figure('Color', 'w');
hold on;
grid on;
box on;

plot(problem.trajectory.t, valid_markers_init, ...
    'Color', [0.6 0.6 0.6], 'LineWidth', 1.5);

plot(problem.trajectory.t, valid_markers_opt, ...
    'Color', [0.2 0.45 0.85], 'LineWidth', 1.5);

yline(problem.M_req, '--r', ...
    'LineWidth', 1.4, ...
    'Label', 'M_{req}');

xlabel('Time (s)');
ylabel('Number of Valid Markers');
title('Rigid-Body Trackability Along the Trajectory');

legend({'Initial Layout', 'Optimized Layout', 'Tracking Requirement'}, ...
    'Location', 'best');

set(gca, 'FontSize', 11);

%% Plot camera layouts

plotTrajectoryCameraLayout( ...
    best_layout, problem, trackable_opt, ...
    'Optimized Camera Layout');

% Optional comparison:
plotTrajectoryCameraLayout( ...
    initial_layout, problem, trackable_init, ...
    'Initial Camera Layout');

%% Save layouts and plotting data

results_folder = fullfile(common_folder, 'saved_layouts');

if ~isfolder(results_folder)
    mkdir(results_folder);
end

saved_run.best_layout    = best_layout;
saved_run.initial_layout = initial_layout;

% Minimum problem data required for future plotting
saved_run.plot_problem.workspace_bounds = problem.workspace_bounds;
saved_run.plot_problem.camera            = problem.camera;
saved_run.plot_problem.trajectory        = problem.trajectory;
saved_run.plot_problem.markers_world     = problem.markers_world;

% Tracking results
saved_run.trackable_opt   = trackable_opt;
saved_run.trackable_init  = trackable_init;
saved_run.valid_markers_opt  = valid_markers_opt;
saved_run.valid_markers_init = valid_markers_init;

% Optimization information
saved_run.output         = output;
saved_run.total_time_sec = total_time_sec;
saved_run.num_cams       = problem.num_cams;
saved_run.k_req          = problem.k_req;
saved_run.M_req          = problem.M_req;
saved_run.random_seed    = problem.optimization.random_seed;
saved_run.date_created   = datetime('now');

% Create a unique filename
timestamp = char(datetime('now', ...
    'Format', 'yyyyMMdd_HHmmss'));

results_file = fullfile( ...
    results_folder, ...
    ['camera_layout_' timestamp '.mat']);

save(results_file, 'saved_run');

fprintf('\nSaved camera layouts to:\n%s\n', results_file);