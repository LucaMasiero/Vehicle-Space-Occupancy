function [p,q,m] = selectFeatures_p2(imgPath, cameraIntrinsics, DLT)
    % SELECTFEATURES_P2 - Allows the user to select two feature points in the
    % given image and plot the chosen points plus the principal camera point
    % on the image
    %
    % Syntax:  points = selectFeatures(imgPath)
    %
    % Inputs:
    %   imgPath - String containing the absolute path to the image file
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
    image= imread(imgPath);             % load image
    [image, ~] = imresize(image, cameraIntrinsics.ImageSize);
    
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

    plotFeaturesOnImage_p2(imgPath, p, q, m, cameraIntrinsics, DLT, true);
end