function plotFeaturesOnImage_p1(imgPath, x, y, cameraIntrinsics, keepFigure)
% PLOTFEATURESONIMAGE_P1 - plot u1,u2,u3 and u4 on the img whose path is
% given as input parameter.
%
% Inputs:
%   imgPath - str path to the image
%   x - [4,1] vector containing x coordinates of each one of the feature
%       points in clockwise order (left taillight - right taillight - bottom-right
%       corner of the license plate - bottom-left corner of the license plate)
%   y - [4,1] vector containing y coordinates of each one of the feature
%       points in the same order as x
%   cameraIntrinsics - structure containing camera intrinsic parameters
%   keepFigure - boolean specifying if theere is a figure we should draw
%       on (true), or just create a new one (false)
%
    
    if ~keepFigure
        figure
    end

    img = imread(imgPath);
    [img, ~] = imresize(img, cameraIntrinsics.ImageSize);
    [img, ~] = undistortImage(img, cameraIntrinsics);
    imshow(im2gray(img))
    hold on
    dy = -20;
    
    % Get principal point
    pp = cameraIntrinsics.PrincipalPoint;

    % Draw feature points
    plot(x, y, 'ro', 'MarkerSize', 10, 'LineWidth', 2);
    % Draw lines between feature points
    line(x(1:2),y(1:2), "Color",'r');
    line(x(2:3),y(2:3), "Color",'r');
    line(x(3:4),y(3:4), "Color",'r');
    line([x(4),x(1)],[y(4),y(1)], "Color",'r');
    % Draw principal point
    plot(pp(1), pp(2), 'g*', 'MarkerSize', 10, 'LineWidth', 2);
    
    % Write feature points and principal point labels
    labels = ["u1", "u2", "u3", "u4"];
    text(x, y + dy, labels, "HorizontalAlignment","center", ...
                            "VerticalAlignment","bottom", ...
                            "Color",'r', ...
                            'FontSize',20, ...
                            'FontWeight','bold');
    text(pp(1), pp(2)+dy, "Principal Point", ...
                          "HorizontalAlignment","center", ...
                          "VerticalAlignment","bottom", ...
                          "Color",'g')
end