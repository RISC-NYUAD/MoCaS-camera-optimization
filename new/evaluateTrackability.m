function [trackable, valid_markers, marker_validity] = ...
    evaluateTrackability(visible_counts, k_req, M_req)
%EVALUATETRACKABILITY Evaluate rigid-body tracking along the trajectory.
%
% Inputs:
%   visible_counts : K x M number of cameras viewing each marker
%   k_req          : cameras required for a marker to be valid
%   M_req          : valid markers required for rigid-body tracking
%
% Outputs:
%   trackable      : K x 1 logical tracking indicator
%   valid_markers  : K x 1 number of valid markers
%   marker_validity: K x M logical marker-validity matrix

    if isempty(visible_counts) || ndims(visible_counts) ~= 2
        error('visible_counts must be a nonempty K x M matrix.');
    end

    if k_req < 1 || k_req ~= round(k_req)
        error('k_req must be a positive integer.');
    end

    if M_req < 1 || M_req ~= round(M_req)
        error('M_req must be a positive integer.');
    end

    number_markers = size(visible_counts, 2);

    if M_req > number_markers
        error('M_req cannot exceed the total number of markers.');
    end

    %% Marker-level multi-view validity

    marker_validity = visible_counts >= k_req;

    %% Number of valid markers at every trajectory sample

    valid_markers = sum(marker_validity, 2);

    %% Rigid-body trackability at every trajectory sample

    trackable = valid_markers >= M_req;
end