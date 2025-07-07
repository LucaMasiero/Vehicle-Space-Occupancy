function [x,y] = selectFeatures(imgPath, cameraIntrinsics)
% SELECTFEATURES - Allows user to select 4 feature points from an image and
% plots the chosen points plus the principal camera point on the image,
% highlighting the quadrangular shape on the rear of the car
%
% Syntax:  points = selectFeatures(imgPath)
%
% Inputs:
%   imgPath - String containing the absolute path to the image file
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
    
    img = imread(imgPath);             % load image
    [img, ~] = imresize(img, cameraIntrinsics.ImageSize);
    pp = cameraIntrinsics.PrincipalPoint;   % principal point
    
    % Undistort image
    [J, ~] = undistortImage(img, cameraIntrinsics);
    imshow(J)
    hold on;
    u1 = drawpoint;
    u2 = drawpoint;
    u3 = drawpoint;
    u4 = drawpoint;
    x = [u1.Position(1);
         u2.Position(1);
         u3.Position(1);
         u4.Position(1)];
    y = [u1.Position(2);
         u2.Position(2);
         u3.Position(2);
         u4.Position(2)];
    
    plotFeaturesOnImage_p1(imgPath, x, y, cameraIntrinsics, true);
end