function P = selectFeatures_p2(image_path, cameraIntrinsics)
% SELECTFEATURES_P2 - Allows the user to select two feature points in the
% given image and plot the chosen points plus the principal camera point
% on the image
%
% Syntax:  points = selectFeatures(image_path)
%
% Inputs:
%   image_path - String containing the absolute path to the image file
%   cameraIntrinsics - structure containing intrinsic parameters of the
%                      calibrated camera
%        
% Outputs:
%   points - 2x2 matrix where each row contains [x, y] coordinates of a point
%
% Other m-files required: none
%

figure()
image= imread(image_path);             % load image
pp = cameraIntrinsics.PrincipalPoint;   % principal point

% Undistort image
[J, ~] = undistortImage(image, cameraIntrinsics);
imshow(J)
hold on;
[x, y] = ginput(2);     % get feature points

% Plot feature points
imshow(im2gray(J))
hold on
plot(x, y, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
plot(pp(1), pp(2), 'g*', 'MarkerSize', 10, 'LineWidth', 2);
line(x(1:2),y(1:2), "Color",'r');

dy = -20;
labels = ["p", "q"];
text(x, y + dy, labels, "HorizontalAlignment","center", "VerticalAlignment","bottom", "Color",'r', 'FontSize',20, 'FontWeight','bold');
text(pp(1), pp(2)+dy, "Principal Point", "HorizontalAlignment","center", "VerticalAlignment","bottom", "Color",'g')

P = [x,y];
end