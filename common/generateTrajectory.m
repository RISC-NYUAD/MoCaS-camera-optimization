function trajectory = generateTrajectory(duration, dt)
%GENERATETRAJECTORY Generate the prescribed OmniOcta trajectory.
%
% Position: [0,-5,2] to [0,0,2]
% Roll:      0 degrees
% Yaw:       0 degrees
% Pitch:     0 to 90 degrees
%
% Inputs:
%   duration : trajectory duration in seconds
%   dt       : sampling interval in seconds
%
% Output:
%   trajectory.t     : K x 1
%   trajectory.p     : K x 3
%   trajectory.R     : 3 x 3 x K
%   trajectory.roll  : K x 1
%   trajectory.pitch : K x 1
%   trajectory.yaw   : K x 1

    if nargin < 1
        duration = 5;
    end

    if nargin < 2
        dt = 0.05;
    end

    %% Time

    trajectory.t = (0:dt:duration)';
    K = numel(trajectory.t);

    s = trajectory.t / duration;

    %% Position

    start_position = [-6.5, -6.5, 2];
    final_position = [6.5,  6.5, 1];

    trajectory.p = (1 - s) .* start_position + ...
                        s  .* final_position;

    %% Orientation

    trajectory.roll  = zeros(K,1);
    trajectory.pitch = deg2rad(90 * s);
    trajectory.yaw   = zeros(K,1);

    trajectory.R = zeros(3,3,K);

    for k = 1:K
        theta = trajectory.pitch(k);

        % Body-to-world rotation.
        % Positive pitch rotates the drone nose upward.
        trajectory.R(:,:,k) = ...
            [cos(theta), 0, -sin(theta);
             0,          1,  0;
             sin(theta), 0,  cos(theta)];
    end
end