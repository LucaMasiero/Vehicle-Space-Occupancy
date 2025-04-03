clear
close all
addpath("matlab\functions\")

iPhone = false;
manualExtraction = true;

% Feature extraction
pyenv('Version', "C:\Users\luca1\miniconda3\envs\py3.12\python.exe");
%addpath(fileparts(fullfile(pwd, "./matlab/scripts/main.m")))

if iPhone
    img_path = "./imgs/pandina/iPhone/panda_3.jpg";
else
    img_path = "./imgs/ibiza/nothing2a/seat_1.jpg";
end

if manualExtraction
    original_image_points = selectFeatures(img_path)';
else
    py_command = "./python/scripts/feature_extraction.py '-file' "+img_path;
    [taillights_centers, plate_corners] = pyrunfile(py_command, ["taillights_centers", "plate_corners"]);
    
    bb1_center = int64(taillights_centers{1});
    bb2_center = int64(taillights_centers{2});
    
    % order between left and right taillights
    if(bb1_center(1) < bb2_center(1))
        left_taillight_center = bb1_center;
        right_taillight_center = bb2_center;
    else
        left_taillight_center = bb2_center;
        right_taillight_center = bb1_center;
    end 
    
    plate_ul = int64(plate_corners{1});
    plate_ur = int64(plate_corners{2});

    original_image_points = double( [left_taillight_center;
                            right_taillight_center;
                            plate_ul;
                            plate_ur]);
end

%% Localization
undistortion = false;
normalize = true;

if iPhone
    load("./matlab/data/iPhone_camera_parameters.mat")
else
    load("./matlab/data/nothing2a_camera_parameters.mat")
end
K = cameraParams.Intrinsics.K;
p = cameraParams.Intrinsics.PrincipalPoint;

image_points = original_image_points;
% undistort image points
image_points_undist = undistortPoints(image_points, cameraParams.Intrinsics);


% translate points to principal point reference frame
image_points = translateImagePoints(image_points, p);

[img_height, img_width, ~] = size(imread(img_path));
K = [1 0 p(1); 0 1 -(img_height-p(2)); 0 0 1]*[0 -1 0; 1 0 0; 0 0 1]*K; %to account for the reference frame used 
                                                             % for calibration and for the image points
K = K./K(3,3);
%K(:,1) = -K(:,1); %to account for the fact that x-cam and x-img are in opposite directions
%(:,2) = -K(:,2); %to account for the fact that y-cam and y-img are in opposite directions

% unit of measure [mm]
if iPhone
    % measures of pandina
    taillights_d = 1210;
    plate_length = 530;
    height = 240;  % between lights' center and plate
else
    % measures of ibiza
    taillights_d = 1040;
    plate_length = 530;
    height = 300;  % between lights' center and plate
end

% Define worlds points as you like but with the same right-hand rule as the camera frame
% x to the left
% y upward
world_points = [taillights_d/2, 0;
                -taillights_d/2, 0;
                plate_length/2, -height;
                -plate_length/2, -height];

% x to the right
% y downward
%world_points = [-taillights_d/2, 0;
%                taillights_d/2, 0;
%                -plate_length/2, height;
%                plate_length/2, height];

if normalize
    % Normalize image and world points for numerical stability
    [image_points, T_img] = normalizePoints(image_points);
    [world_points, T_world] = normalizePoints(world_points);
end

% compute homography matrix
if undistortion
    image_points_undist = undistortPoints(image_points, cameraParams.Intrinsics);
    if normalize
        [image_points_undist, T_img] = normalizePoints(image_points_undist);
    end
    H = dlt([world_points'; 1 1 1 1], [image_points_undist'; 1 1 1 1]);
else
    H = dlt([world_points'; 1 1 1 1], [image_points'; 1 1 1 1]);
end

if normalize
    H = denormalizeHomography(H, T_img, T_world);
end
H = H./H(3,3);
h1 = H(:,1);
h2 = H(:,2);
h3 = H(:,3);

% normalization factor.
lambda = 1 / norm(K \ h1); %=1/norm(r1)

% r1 = K^-1 * h1 normalized
r1 = (K \ h1) * lambda;
r2 = (K \ h2) * lambda;
r3 = cross(r1,r2);

% direction of the world axis with respect to the camera frame
R = [r1, r2, r3];
if det(R) == -1
    % negate all columns to ensure proper rotation matrix
    R = -R;
end

% due to noise in the data R may be not a true rotation matrix.
% approximate it through svd, obtaining an orthogonal matrix
%R(1,:) = -R(1,:);
%R(2,:) = -R(2,:);
[U, ~, V] = svd(R);
R = U * V';

% Compute translation vector. This vector is the position of the plane wrt
% the reference frame of the camera.
T = (K \ (lambda * h3))

cameraRotation = R.'

% since T is expressed in the camera reference frame we want it in the plane
% reference frame where R.' is the rotation of the camera wrt the plane
cameraPosition = -R.' * T


%% Plot result with plotly

%% Using Matlab function 'estworldpose'
undistortedPoints = undistortPoints(image_points,cameraParams.Intrinsics);
worldPose = estworldpose(undistortedPoints,world_points,cameraParams.Intrinsics);

pcshow(worldPoints,VerticalAxis="Y",VerticalAxisDir="down", ...
    MarkerSize=30);
hold on
plotCamera(Size=10,Orientation=worldPose.R', ...
    Location=worldPose.Translation);
hold off
