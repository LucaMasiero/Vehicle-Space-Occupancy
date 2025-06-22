function ray = backprojectionRay(p, K)
    % BACKPROJECTIONRAY    Compute the backprojection ray of point p through
    % camera calibration K. p is a 2D homogeneous vector (x,y,1) and K is a 3x3
    % matrix defining the intrinsic parameters of the camera.
    % 
    % N.B. K can be retrieved from the calibration parameters of the camera as
    % follows: cameraParams.Intrinsics.K

    p_dir = inv(K)*p;
    ray = p_dir/norm(p_dir);
end