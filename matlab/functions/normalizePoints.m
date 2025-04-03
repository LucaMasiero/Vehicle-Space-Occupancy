function [normalized_points, T] = normalizePoints(points)
% normalizePoints Normalize a set of 2D points to improve numerical stability
%
% Input:
%   points - Nx2 matrix where each row is [x, y] coordinates of a point
%
% Output:
%   normalized_points - Nx2 matrix of normalized points
%   T - 3x3 normalization matrix such that [x' y' 1]' = T * [x y 1]'
%
% The normalization translates the centroid to the origin and
% scales points so their average distance from origin is sqrt(2)

% Check input format
if size(points, 2) ~= 2
    error('Points matrix must have 2 columns [x, y]');
end

% Calculate the centroid
centroid = mean(points, 1);

% Shift points to have centroid at origin
shifted_points = points - centroid;

% Calculate the average distance from origin
distances = sqrt(sum(shifted_points.^2, 2));
avg_distance = mean(distances);

% Calculate the scaling factor
scale = sqrt(2) / avg_distance;

% Create the normalization matrix
T = [
    scale, 0, -scale*centroid(1);
    0, scale, -scale*centroid(2);
    0, 0, 1
];

% Apply normalization to the points
homogeneous_points = [points, ones(size(points, 1), 1)];
normalized_homogeneous = (T * homogeneous_points')';

% Convert back to 2D coordinates
normalized_points = normalized_homogeneous(:, 1:2);

% Display information about the normalization
fprintf('Normalization applied:\n');
fprintf('  Original centroid: (%.4f, %.4f)\n', centroid(1), centroid(2));
fprintf('  Original avg distance: %.4f\n', avg_distance);
fprintf('  Scaling factor: %.4f\n', scale);

% Verify normalization
new_centroid = mean(normalized_points, 1);
new_distances = sqrt(sum(normalized_points.^2, 2));
new_avg_distance = mean(new_distances);
fprintf('  New centroid: (%.6f, %.6f)\n', new_centroid(1), new_centroid(2));
fprintf('  New avg distance: %.6f\n', new_avg_distance);
end