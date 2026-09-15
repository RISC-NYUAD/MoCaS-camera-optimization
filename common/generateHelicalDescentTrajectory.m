function trajectory = generateHelicalDescentTrajectory(duration, dt)
%GENERATEHELICALDESCENTTRAJECTORY Generate a descending helical trajectory.
%
% The drone completes two turns around the workspace centre while
% descending from z = 5.5 m to z = 0.75 m. Roll and pitch vary smoothly,
% while yaw remains zero so that the effects of roll and pitch are isolated.
%
% The complete trajectory remains inside:
%   x: [-8, 8] m, y: [-8, 8] m, z: [0, 6] m
%
% Inputs:
%   duration : trajectory duration in seconds
%   dt       : sampling interval in seconds
%
% Output fields:
%   trajectory.t     : K x 1 time vector
%   trajectory.p     : K x 3 position [x,y,z]
%   trajectory.R     : 3 x 3 x K body-to-world rotation matrices
%   trajectory.roll  : K x 1 roll angle in radians
%   trajectory.pitch : K x 1 pitch angle in radians
%   trajectory.yaw   : K x 1 yaw angle in radians

    if nargin < 1 || isempty(duration)
        duration = 10;
    end

    if nargin < 2 || isempty(dt)
        dt = 0.05;
    end

    validateattributes(duration, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'positive'}, mfilename, 'duration');
    validateattributes(dt, {'numeric'}, ...
        {'scalar', 'real', 'finite', 'positive'}, mfilename, 'dt');

    %% Time

    trajectory.t = (0:dt:duration)';

    if trajectory.t(end) < duration
        trajectory.t(end+1,1) = duration;
    end

    K = numel(trajectory.t);
    s = trajectory.t / duration;

    %% Helical position

    helix_radius   = 3.0;     % metres
    number_turns   = 2;
    initial_height = 4.5;     % metres
    final_height   = 2.0;    % metres

    helix_angle = 2*pi*number_turns*s;

    trajectory.p = [ ...
        helix_radius * cos(helix_angle), ...
        helix_radius * sin(helix_angle), ...
        initial_height + (final_height - initial_height) * s];

    %% Varying orientation

    maximum_roll  = deg2rad(30);
    maximum_pitch = deg2rad(60);

    % Roll changes once per helical turn. Pitch changes twice per turn,
    % producing different attitude combinations around the helix.
    trajectory.roll  = maximum_roll  * sin(helix_angle);
    trajectory.pitch = maximum_pitch * sin(2 * helix_angle);
    trajectory.yaw   = zeros(K,1);

    %% Body-to-world rotation matrices

    trajectory.R = zeros(3,3,K);

    for k = 1:K
        trajectory.R(:,:,k) = bodyToWorldRotation( ...
            trajectory.roll(k), ...
            trajectory.pitch(k), ...
            trajectory.yaw(k));
    end
end


function R = bodyToWorldRotation(roll, pitch, yaw)
%BODYTOWORLDROTATION Preserve the pitch convention of generateTrajectory.

    cr = cos(roll);
    sr = sin(roll);
    cp = cos(pitch);
    sp = sin(pitch);
    cy = cos(yaw);
    sy = sin(yaw);

    R_roll = [ ...
        1,  0,   0;
        0, cr, -sr;
        0, sr,  cr];

    % This sign convention matches the original trajectory function.
    R_pitch = [ ...
         cp, 0, -sp;
          0, 1,   0;
         sp, 0,  cp];

    R_yaw = [ ...
        cy, -sy, 0;
        sy,  cy, 0;
         0,   0, 1];

    R = R_yaw * R_pitch * R_roll;
end
