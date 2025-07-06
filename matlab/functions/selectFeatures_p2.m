function [p,q,m] = selectFeatures_p2(image_path, cameraIntrinsics, DLT)
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
    %   DLT - boolean value telling if we need to extract features for DLT
    %   homography estimation (true) or not (false)
    %        
    % Outputs:
    %   points - 2x2 matrix where each row contains [x, y] coordinates of a point
    %
    % Other m-files required: none
    %

    p = zeros(3,2);
    q = zeros(3,2);
    m = zeros(3,4);

    figure()
    image= imread(image_path);             % load image
    [image, ~] = imresize(image, cameraIntrinsics.ImageSize);
    pp = cameraIntrinsics.PrincipalPoint;   % principal point
    
    % Undistort image
    [J, ~] = undistortImage(image, cameraIntrinsics);
    imshow(J)
    hold on;
    p = drawpoint;
    q = drawpoint;
    p = [p.Position(1); p.Position(2); 1];
    q = [q.Position(1); q.Position(2); 1];

    if DLT
        % homography features
        [mx, my] = getpts();
        m(:,1) = [mx(1); my(1); 1];
        m(:,2) = [mx(2); my(2); 1];
        m(:,3) = [mx(3); my(3); 1];
        m(:,4) = [mx(4); my(4); 1];
    end

    % Plot feature points
    imshow(im2gray(J))
    hold on
    dy = -25;
    dx = 25;
    MRK_SZ = 20;

    % draw car feature points
    scatter(p(1), p(2), MRK_SZ,'o','filled', 'MarkerEdgeColor','r', 'MarkerFaceColor','r');
    scatter(q(1), q(2), MRK_SZ,'o','filled', 'MarkerEdgeColor','r', 'MarkerFaceColor','r');
    line([p(1),q(1)] ,[p(2),q(2)], "Color",'r');
    % draw camera principal point
    scatter(pp(1), pp(2), 2*MRK_SZ,'+','filled', 'MarkerEdgeColor','c', 'MarkerFaceColor','c');
    % write car features labels
    text(p(1)-4*dx, p(2)+dy, 'p', 'FontSize', MRK_SZ, 'Color', 'r', 'FontWeight','bold');
    text(q(1)+dx, q(2)+dy, 'q', 'FontSize', MRK_SZ, 'Color', 'r', 'FontWeight','bold');
    text(pp(1)+dx, pp(2)+dy, "Principal Point", 'FontSize', MRK_SZ, "Color",'c');
    
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