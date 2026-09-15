function trajectory = generateForwardOscillatingTrajectory(duration, dt)
%GENERATEFORWARDOSCILLATINGTRAJECTORY Generate a forward weaving trajectory.
%
% The drone travels forward along the positive y direction while completing
% four left-right oscillations in x. During each complete lateral
% oscillation, pitch moves smoothly from one attitude extreme to the other:
% -90 to +90 degrees, then +90 to -90 degrees. Altitude, roll and yaw remain
% constant.
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

    %% Forward and lateral motion

    initial_y        = -5.5;
    final_y          =  5.5;
    lateral_amplitude = 3.0;
    flight_height     = 3.0;
    number_oscillations = 3;

    lateral_phase = 2*pi*number_oscillations*s;

    x = lateral_amplitude * sin(lateral_phase);
    y = initial_y + (final_y - initial_y) * s;
    z = flight_height * ones(K,1);

    trajectory.p = [x, y, z];

    %% Pitch sweep associated with each complete lateral oscillation

    % Pitch advances through half a cosine cycle during each complete
    % lateral oscillation. Therefore:
    %   lateral cycle 1: -90 to +90 degrees
    %   lateral cycle 2: +90 to -90 degrees
    % and the pattern repeats without any attitude discontinuity.
    pitch_phase = 2*pi*number_oscillations*s;

    trajectory.roll  = zeros(K,1);
    trajectory.pitch = -deg2rad(90) * cos(pitch_phase);
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