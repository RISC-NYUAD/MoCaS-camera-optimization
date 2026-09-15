clc;
clear;
close all;

%% Paths

common_folder = ...
    'C:\Users\ama10362\Desktop\NYU\Vicon markers paper\codes\icara2027\common';

addpath(common_folder);

results_file = fullfile( ...
    common_folder, ...
    'saved_layouts', ...
    'Oscillatinghpcsuccess.mat');

if ~isfile(results_file)
    error('Results file was not found:\n%s', results_file);
end

%% Load saved results

loaded_data = load(results_file, 'saved_run');

if ~isfield(loaded_data, 'saved_run')
    error('The selected MAT-file does not contain saved_run.');
end

saved_run = loaded_data.saved_run;

fprintf('\nLoaded results from:\n%s\n', results_file);

%% Display all available saved fields

fprintf('\n===== AVAILABLE SAVED DATA =====\n');
disp(fieldnames(saved_run));

fprintf('\n===== OPTIMIZATION OUTPUT =====\n');
disp(saved_run.output);

%% General run information

fprintf('\n===== SAVED RUN INFORMATION =====\n');

if isfield(saved_run, 'date_created')
    fprintf('Date created                  : %s\n', ...
        char(saved_run.date_created));
end

if isfield(saved_run, 'random_seed')
    fprintf('Random seed                  : %d\n', ...
        saved_run.random_seed);
end

fprintf('Number of cameras            : %d\n', ...
    saved_run.num_cams);

fprintf('Required views per marker    : %d\n', ...
    saved_run.k_req);

fprintf('Required valid markers       : %d\n', ...
    saved_run.M_req);

fprintf('Optimization time            : %.2f s (%.2f min)\n', ...
    saved_run.total_time_sec, ...
    saved_run.total_time_sec / 60);

if isfield(saved_run.output, 'iterations')
    fprintf('PSO iterations               : %d\n', ...
        saved_run.output.iterations);
end

if isfield(saved_run.output, 'funccount')
    fprintf('Objective evaluations        : %d\n', ...
        saved_run.output.funccount);
end

fprintf('=================================\n');

%% Display camera-layout values

fprintf('\n===== INITIAL CAMERA LAYOUT =====\n');
disp(saved_run.initial_layout);

fprintf('\n===== OPTIMIZED CAMERA LAYOUT =====\n');
disp(saved_run.best_layout);

%% Tracking statistics

trackable_init = logical(saved_run.trackable_init(:));
trackable_opt  = logical(saved_run.trackable_opt(:));

valid_markers_init = saved_run.valid_markers_init(:);
valid_markers_opt  = saved_run.valid_markers_opt(:);

t = saved_run.plot_problem.trajectory.t(:);
K = numel(t);

failure_samples_init = find(~trackable_init);
failure_samples_opt  = find(~trackable_opt);

fprintf('\n===== TRACKING RESULTS =====\n');
fprintf('Trajectory samples           : %d\n', K);

fprintf('\nInitial layout:\n');
fprintf('Trackable samples            : %d/%d\n', ...
    sum(trackable_init), K);
fprintf('Tracking percentage          : %.2f %%\n', ...
    100 * mean(trackable_init));
fprintf('Minimum valid markers        : %d\n', ...
    min(valid_markers_init));
fprintf('Mean valid markers           : %.2f\n', ...
    mean(valid_markers_init));
fprintf('Failure samples              : %d\n', ...
    numel(failure_samples_init));

fprintf('\nOptimized layout:\n');
fprintf('Trackable samples            : %d/%d\n', ...
    sum(trackable_opt), K);
fprintf('Tracking percentage          : %.2f %%\n', ...
    100 * mean(trackable_opt));
fprintf('Minimum valid markers        : %d\n', ...
    min(valid_markers_opt));
fprintf('Mean valid markers           : %.2f\n', ...
    mean(valid_markers_opt));
fprintf('Failure samples              : %d\n', ...
    numel(failure_samples_opt));

fprintf('==============================\n');

%% Display failure times

if ~isempty(failure_samples_init)
    fprintf('\nInitial-layout failure times (s):\n');
    disp(t(failure_samples_init).');
else
    fprintf('\nThe initial layout has no tracking failures.\n');
end

if ~isempty(failure_samples_opt)
    fprintf('\nOptimized-layout failure times (s):\n');
    disp(t(failure_samples_opt).');
else
    fprintf('The optimized layout has no tracking failures.\n');
end

%% Plot valid markers over time

figure('Color', 'w', ...
       'Name', 'Saved Tracking Results');

hold on;
grid on;
box on;

plot(t, valid_markers_init, ...
    'Color', [0.60 0.60 0.60], ...
    'LineWidth', 1.8);

plot(t, valid_markers_opt, ...
    'Color', [0.20 0.45 0.85], ...
    'LineWidth', 1.8);

yline(saved_run.M_req, '--r', ...
    'LineWidth', 1.5, ...
    'Label', 'M_{req}', ...
    'LabelHorizontalAlignment', 'left');

xlabel('Time (s)');
ylabel('Number of Valid Markers');
title('Trackability Along the Saved Trajectory');

legend( ...
    {'Initial layout', ...
     'Optimized layout', ...
     'Tracking requirement'}, ...
    'Location', 'best');

set(gca, 'FontSize', 12);

%% Plot tracking status

figure('Color', 'w', ...
       'Name', 'Saved Tracking Status');

stairs(t, double(trackable_init), ...
    'Color', [0.60 0.60 0.60], ...
    'LineWidth', 1.8);

hold on;
grid on;
box on;

stairs(t, double(trackable_opt), ...
    'Color', [0.20 0.45 0.85], ...
    'LineWidth', 1.8);

xlabel('Time (s)');
ylabel('Tracking Status');
title('Rigid-Body Tracking Status');

yticks([0 1]);
yticklabels({'Not trackable', 'Trackable'});
ylim([-0.1 1.1]);

legend({'Initial layout', 'Optimized layout'}, ...
    'Location', 'best');

set(gca, 'FontSize', 12);

%% Plot optimized and initial camera layouts

plotTrajectoryCameraLayout( ...
    saved_run.best_layout, ...
    saved_run.plot_problem, ...
    saved_run.trackable_opt, ...
    'Saved Optimized Camera Layout');

plotTrajectoryCameraLayout( ...
    saved_run.initial_layout, ...
    saved_run.plot_problem, ...
    saved_run.trackable_init, ...
    'Saved Initial Camera Layout');