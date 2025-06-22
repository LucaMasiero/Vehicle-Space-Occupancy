function [x,y] = selectFeatures(image_path, cameraIntrinsics)
% SELECTFEATURES - Allows user to select 4 feature points from an image and
% plots the chosen points plus the principal camera point on the image,
% highlighting the quadrangular shape on the rear of the car
%
% Syntax:  points = selectFeatures(image_path)
%
% Inputs:
%   image_path - String containing the absolute path to the image file
%   cameraIntrinsics - structure containing intrinsic parameters of the
%                      calibrated camera
%        
% Outputs:
%   points - 4x2 matrix where each row contains [x, y] coordinates of a point
%
% Example:
%   points = selectFourPoints('myimage.jpg');
%
% Other m-files required: none
%

image = imread(image_path);             % load image
pp = cameraIntrinsics.PrincipalPoint;   % principal point

% Undistort image
[J, ~] = undistortImage(image, cameraIntrinsics);
imshow(J)
hold on;
[x, y] = ginput(4);     % get feature points

% Plot feature points
imshow(im2gray(J))
hold on
plot(x, y, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
plot(pp(1), pp(2), 'g*', 'MarkerSize', 10, 'LineWidth', 2);
line(x(1:2),y(1:2), "Color",'r');
line(x(2:3),y(2:3), "Color",'r');
line(x(3:4),y(3:4), "Color",'r');
line([x(4),x(1)],[y(4),y(1)], "Color",'r');

dy = -20;
labels = ["UL", "UR", "BR", "BL"];
labels = ["u1", "u2", "u3", "u4"];
text(x, y + dy, labels, "HorizontalAlignment","center", "VerticalAlignment","bottom", "Color",'r', 'FontSize',20, 'FontWeight','bold');
text(pp(1), pp(2)+dy, "Principal Point", "HorizontalAlignment","center", "VerticalAlignment","bottom", "Color",'g')

end