function [p,q,m] = selectFeatures_p2(image_path, cameraIntrinsics, DLT_features)
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

    if DLT_features
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
    dy = -20;
    FNT_SZ = 20;

    % draw car feature points
    plot(p(1), p(2), 'ro', 'MarkerSize', 5, 'LineWidth', 2);
    plot(q(1), q(2), 'ro', 'MarkerSize', 5, 'LineWidth', 2);
    plot(pp(1), pp(2), 'g*', 'MarkerSize', 10, 'LineWidth', 2);
    line([p(1),q(1)] ,[p(2),q(2)], "Color",'r');
    % car features labels
    text(p(1), p(2) + dy, 'p', 'FontSize', FNT_SZ, 'Color', 'r', 'FontWeight','bold');
    text(q(1), q(2) + dy, 'q', 'FontSize', FNT_SZ, 'Color', 'r', 'FontWeight','bold');
    text(pp(1), pp(2)+dy, "Principal Point", 'FontSize', FNT_SZ, "Color",'g');
    
    if DLT_features
        % draw homography feature points
        plot(m(1,1), m(2,1),'o', 'Color','y');
        plot(m(1,2), m(2,2),'o','Color', 'y');
        plot(m(1,3), m(2,3),'o','Color', 'y');
        plot(m(1,4), m(2,4),'o','Color', 'y');
        
        % homography features labels
        text(m(1,1), m(2,1), 'a''', 'FontSize', FNT_SZ, 'Color', 'r');
        text(m(1,2), m(2,2), 'b''', 'FontSize', FNT_SZ, 'Color', 'r');
        text(m(1,3), m(2,3), 'c''', 'FontSize', FNT_SZ, 'Color', 'r');
        text(m(1,4), m(2,4), 'd''', 'FontSize', FNT_SZ, 'Color', 'r');
    end    
end