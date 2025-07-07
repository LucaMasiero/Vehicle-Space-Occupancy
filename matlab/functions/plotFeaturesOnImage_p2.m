function plotFeaturesOnImage_p2(imgPath, p, q, m, cameraIntrinsics, DLT, keepFigure)
% PLOTFEATURESONIMAGE - plot p,q and DLT features on the img whose path is
% given as input parameter. If DLT is not currently used, then the features
% contained in m, if any, are not plotted on the image. This creates a new
% figure every time it is called.
%
% Inputs:
%   imgPath - str path to the image
%   p - [3,1] vector containing [x;y;1] homogeneous coordinates of feature point p
%       (i.e., left taillight)
%   q - [3,1] vector containing [x;y;1] homogeneous coordinates of feature point q
%       (i.e., right taillight)
%   m - [3,4] matrix containing four features for DLT homography estimation
%       (3 coordinates: [x;y;1]; 4 points)
%   cameraIntrinsics - structure containing camera intrinsic parameters 
%   DLT - boolean variable telling us if the user wants to apply DLT or not; 
%           if true then plot DLT features contained in m
%           if false then do not plot DLT features contained in m
%   keepFigure - boolean telling us if theere is a figure we should draw
%       on (true), or just create a new one (false)
%
    % Plot feature points
    if ~keepFigure
        figure
    end
    img = imread(imgPath);
    [img, ~] = imresize(img, cameraIntrinsics.ImageSize);
    imshow(rgb2gray(img))
    hold on
    dy = -25;
    dx = 25;
    MRK_SZ = 20;
    
    % Get principal point
    pp = cameraIntrinsics.PrincipalPoint;

    % draw car feature points
    scatter(p(1), p(2), MRK_SZ,'o','filled', 'MarkerEdgeColor','r', 'MarkerFaceColor','r');
    scatter(q(1), q(2), MRK_SZ,'o','filled', 'MarkerEdgeColor','r', 'MarkerFaceColor','r');
    line([p(1),q(1)] ,[p(2),q(2)], "Color",'r');
    % draw camera principal point
    scatter(pp(1), pp(2), 2*MRK_SZ,'+','filled', 'MarkerEdgeColor','c', 'MarkerFaceColor','c');
    % write car features labels
    text(p(1)-4*dx, p(2)+dy, 'p', 'FontSize', MRK_SZ, 'Color', 'r', 'FontWeight','bold');
    text(q(1)+dx, q(2)+dy, 'q', 'FontSize', MRK_SZ, 'Color', 'r', 'FontWeight','bold');
    %text(pp(1)+dx, pp(2)+dy, "Principal Point", 'FontSize', MRK_SZ, "Color",'c');
    
    if DLT
        DLT_color = 'g';

        % draw homography feature points
        scatter(m(1,1), m(2,1), MRK_SZ, 'o','filled', 'MarkerEdgeColor', DLT_color, 'MarkerFaceColor', DLT_color);
        scatter(m(1,2), m(2,2), MRK_SZ, 'o','filled', 'MarkerEdgeColor', DLT_color, 'MarkerFaceColor', DLT_color);
        scatter(m(1,3), m(2,3), MRK_SZ, 'o','filled', 'MarkerEdgeColor', DLT_color, 'MarkerFaceColor', DLT_color);
        scatter(m(1,4), m(2,4), MRK_SZ, 'o','filled', 'MarkerEdgeColor', DLT_color, 'MarkerFaceColor', DLT_color);
        % write homography features labels
        text(m(1,1)-4*dx, m(2,1)+dy, 'a', 'FontSize', MRK_SZ, 'Color', DLT_color, 'FontWeight','bold');
        text(m(1,2)-4*dx, m(2,2)+dy, 'b', 'FontSize', MRK_SZ, 'Color', DLT_color, 'FontWeight','bold');
        text(m(1,3)-4*dx, m(2,3)+dy, 'c', 'FontSize', MRK_SZ, 'Color', DLT_color, 'FontWeight','bold');
        text(m(1,4)-4*dx, m(2,4)+dy, 'd', 'FontSize', MRK_SZ, 'Color', DLT_color, 'FontWeight','bold');
    end
end