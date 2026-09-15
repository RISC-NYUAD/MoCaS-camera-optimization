function [best_layout, initial_layout, output] = ...
    optimizeTrajectoryCameraLayout(problem)
%OPTIMIZETRAJECTORYCAMERALAYOUT Optimize camera poses using PSO.
%
% Each camera is represented by:
%   [x, y, z, yaw, pitch]
%
% Inputs are provided through the problem structure.
%
% Outputs:
%   best_layout    : N x 5 optimized camera layout
%   initial_layout : N x 5 initial camera layout
%   output         : PSO information

    workspace_bounds = problem.workspace_bounds;
    num_cams         = problem.num_cams;

    %% Decision variables

    num_variables = 5 * num_cams;

    %% Lower and upper bounds

    position_lower = workspace_bounds(:,1)';
    position_upper = workspace_bounds(:,2)';

    yaw_bounds   = problem.camera.yaw_bounds;
    pitch_bounds = problem.camera.pitch_bounds;

    lower_camera = [position_lower, ...
                    yaw_bounds(1), ...
                    pitch_bounds(1)];

    upper_camera = [position_upper, ...
                    yaw_bounds(2), ...
                    pitch_bounds(2)];

    lower_bounds = repmat(lower_camera, 1, num_cams);
    upper_bounds = repmat(upper_camera, 1, num_cams);

    %% Initial camera layout

    initial_layout = initializeCameras( ...
        workspace_bounds, ...
        problem.trajectory.p, ...
        num_cams);

    initial_vector = reshape(initial_layout', 1, []);

    %% Optimization settings

    if isfield(problem, 'optimization')
        settings = problem.optimization;
    else
        settings = struct();
    end

    if isfield(settings, 'swarm_size')
        swarm_size = settings.swarm_size;
    else
        swarm_size = 200;
    end

    if isfield(settings, 'max_iterations')
        max_iterations = settings.max_iterations;
    else
        max_iterations = 200;
    end

    if isfield(settings, 'random_seed')
        random_seed = settings.random_seed;
    else
        random_seed = 1;
    end

    rng(random_seed, 'twister');

    %% Initial swarm

    initial_swarm = zeros(swarm_size, num_variables);
    initial_swarm(1,:) = initial_vector;

    number_local = floor(swarm_size / 2);
    variable_range = upper_bounds - lower_bounds;

    % Half of the particles start close to the initial layout
    for particle = 2:number_local
        perturbation = 0.1 .* variable_range .* ...
                       randn(1, num_variables);

        candidate = initial_vector + perturbation;

        candidate = max(candidate, lower_bounds);
        candidate = min(candidate, upper_bounds);

        initial_swarm(particle,:) = candidate;
    end

    % Remaining particles are distributed throughout the workspace
    for particle = number_local+1:swarm_size
        initial_swarm(particle,:) = lower_bounds + ...
            rand(1, num_variables) .* variable_range;
    end

    %% Objective function

    objective_function = @(x) ...
        trajectoryLayoutObjective(x, problem);

    %% Particle swarm options

    options = optimoptions('particleswarm', ...
        'SwarmSize', swarm_size, ...
        'MaxIterations', max_iterations, ...
        'InitialSwarmMatrix', initial_swarm, ...
        'UseParallel', true, ...
        'ObjectiveLimit', 1e-12, ...
        'Display', 'iter');

    %% Run PSO

    [optimal_vector, objective_value, exitflag, pso_output] = ...
        particleswarm( ...
            objective_function, ...
            num_variables, ...
            lower_bounds, ...
            upper_bounds, ...
            options);

    %% Convert solution to camera layout

    best_layout = reshape(optimal_vector, [5, num_cams])';

    output = pso_output;
    output.objective_value = objective_value;
    output.exitflag = exitflag;
end


function initial_layout = initializeCameras( ...
    workspace_bounds, trajectory_positions, num_cams)
%INITIALIZECAMERAS Place cameras around the upper workspace perimeter.

    xmin = workspace_bounds(1,1);
    xmax = workspace_bounds(1,2);
    ymin = workspace_bounds(2,1);
    ymax = workspace_bounds(2,2);
    zmax = workspace_bounds(3,2);

    dx = xmax - xmin;
    dy = ymax - ymin;

    perimeter = 2 * (dx + dy);
    camera_height = 0.9 * zmax;

    % Cameras initially look toward the trajectory center
    trajectory_center = mean(trajectory_positions, 1);

    initial_layout = zeros(num_cams, 5);

    for camera_index = 1:num_cams

        distance_on_perimeter = ...
            (camera_index - 1) / num_cams * perimeter;

        if distance_on_perimeter <= dx
            x = xmin + distance_on_perimeter;
            y = ymin;

        elseif distance_on_perimeter <= dx + dy
            x = xmax;
            y = ymin + distance_on_perimeter - dx;

        elseif distance_on_perimeter <= 2*dx + dy
            x = xmax - ...
                (distance_on_perimeter - dx - dy);
            y = ymax;

        else
            x = xmin;
            y = ymax - ...
                (distance_on_perimeter - 2*dx - dy);
        end

        z = camera_height;

        direction = trajectory_center - [x, y, z];

        yaw = atan2(direction(2), direction(1));

        horizontal_distance = norm(direction(1:2));
        pitch = atan2(direction(3), horizontal_distance);

        initial_layout(camera_index,:) = ...
            [x, y, z, yaw, pitch];
    end
end