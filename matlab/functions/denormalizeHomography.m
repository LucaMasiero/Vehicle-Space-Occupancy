function H = denormalizeHomography(H_normalized, T_img, T_world)
% denormalizeHomography Converts a homography from normalized coordinates back to original coordinates
%
% Inputs:
%   H_normalized - 3x3 homography matrix computed using normalized coordinates
%   T_img - 3x3 normalization matrix for image points
%   T_world - 3x3 normalization matrix for world points
%
% Output:
%   H - 3x3 denormalized homography matrix that works with original coordinates
%
% The denormalized homography is computed as: H = inv(T_img) * H_normalized * T_world

% Check inputs
if size(H_normalized, 1) ~= 3 || size(H_normalized, 2) ~= 3
    error('H_normalized must be a 3x3 matrix');
end
if size(T_img, 1) ~= 3 || size(T_img, 2) ~= 3
    error('T_img must be a 3x3 matrix');
end
if size(T_world, 1) ~= 3 || size(T_world, 2) ~= 3
    error('T_world must be a 3x3 matrix');
end

% Compute inverse of image normalization matrix
T_img_inv = inv(T_img);

% Denormalize the homography
H = T_img_inv * H_normalized * T_world;

% Normalize the homography to ensure H(3,3) = 1
H = H / H(3,3);

% Display information about the denormalization
fprintf('Homography denormalization applied\n');
fprintf('Condition number of normalized homography: %.2f\n', cond(H_normalized));
fprintf('Condition number of denormalized homography: %.2f\n', cond(H));
end