close all
clear
addpath("matlab\functions")

% Here you can choose whether:
%   - to save the extracted features,
%   - to use 'precooked' features
%
saveFeatures = false;
UseExamples = true;

% --------------- FEATURE EXTRACTION ---------------
% ATTENTION: it is very important the the features on the car are selected
% in clockwise order:
%   - u1, i.e. left taillight
%   - u2, i.e. right taillitght
%   - u3, i.e. bottom-right corner of license plate
%   - u4, i.e. bottom-left corner of license plate
%

% Choose the camera
% I-PHONE
load("./matlab/data/cameras/iPhone_camera_params.mat") 
K = cameraParams.Intrinsics.K; % camera calibration

if UseExamples
    % Select precooked.mat file for the desired image
    [file,location] = uigetfile({'*.*'; '*.mat'}, 'Folder', what('imgs\pandina\iPhone\').path);
    abs_path = fullfile(location,file);
    % Retrieve features
    feature_points = load(abs_path);
    x = feature_points.x;
    y = feature_points.y;
    disp('Extracted features from precooked file.')

    % Plot feature points on the image
    files = dir(fullfile(location, '*.jpg'));
    imgFile = files(1); %take one image
    imgPath = fullfile(location, imgFile.name);
    plotFeaturesOnImage_p1(imgPath, x, y, cameraParams.Intrinsics, false);

else
    % select image
    [file,location] = uigetfile({'*.*'; '*.jpg'}, 'Folder', what('imgs\pandina\iPhone').path);
    abs_path = fullfile(location,file);
    
    % Extract features
    [x,y] = selectFeatures_p1(abs_path, cameraParams.Intrinsics);

    if saveFeatures
        % Save features if rerquired
        save(location+"\precooked.mat", "x", "y");
    end
end

% --------------- COMPUTE FEATURE POINTS POSITION IN CAMERA REFERENCE FRAME ---------------
UL = [x(1);y(1);1]; % left taillight
UR = [x(2);y(2);1]; % right taillight
BR = [x(3);y(3);1]; % bottom-right corner of plate
BL = [x(4);y(4);1]; % bottom-left corner of plate

tl = cross(UL,UR);  % taillights line
pl = cross(BL,BR);  % license plate line

p_inf = cross(tl,pl);   % image of the point at infinity
p_inf = p_inf/p_inf(3); % homogenize point at infinity

% Backprojection rays
inf_dir = backprojectionRay(p_inf, K);
BL_dir = backprojectionRay(BL, K);
BR_dir = backprojectionRay(BR, K);
UL_dir = backprojectionRay(UL, K);
UR_dir = backprojectionRay(UR, K);

% Pandina: taillights distance=114.5cm; license plate=53cm
[dUL, dUR] = deriveDistance(UL_dir, UR_dir, inf_dir, 114.5);
[dBL, dBR] = deriveDistance(BL_dir, BR_dir, inf_dir, 53);

% Coordinates of the real world points in the camera ref. system
UL_cam = dUL*UL_dir;
UR_cam = dUR*UR_dir;
BL_cam = dBL*BL_dir;
BR_cam = dBR*BR_dir;

% --------------- DERIVE CAMERA ROTATION MATRIX ---------------
X_axis = inf_dir;

% Find another axis perpendicular to X_axis
temp = [0, 1, 0]';                  % Arbitrary vector
if abs(dot(X_axis, temp)) > 0.9     % if too parallel
    temp = [0, 0, 1]';
end
    
Y_axis = cross(temp, X_axis);
Y_axis = Y_axis / norm(Y_axis);

% Z-axis completes the coordinate system
Z_axis = cross(X_axis, Y_axis);

% Form the rotation matrix (reference system of the world w.r.t. camera reference system)
R_world_to_cam = [X_axis, Y_axis, Z_axis];
% Reference system of the camera w.r.t. world reference system
R_cam_to_world = R_world_to_cam';

% Rotate points as the world reference frame to obtain a result 
% which is more similar to what we would expect.
% Remember that the camera Z-axis is pointing towards the principal point
UL_new_cam = R_cam_to_world*UL_cam;
UR_new_cam = R_cam_to_world*UR_cam;
BL_new_cam = R_cam_to_world*BL_cam;
BR_new_cam = R_cam_to_world*BR_cam;

% --------------- PLOT CAMERA REFERENCE FRAME w.r.t. WORLD REFERENCE FRAME ---------------
% We applied R_cam_to_world rotation to move the points in a camera
% reference system that is alligned with the world reference system
% (ideally aligned with the street)

% Take camera reference axis
X = R_cam_to_world(:,1);
Y = R_cam_to_world(:,2);
Z = R_cam_to_world(:,3);

figure()
hold on
title("Camera reference system w.r.t. World reference system")

% Plot CAMERA reference system
quiver3(0,0,0, X(1),X(2),X(3),0.5,'r');
quiver3(0,0,0, Y(1),Y(2),Y(3),0.5,'g');
quiver3(0,0,0, Z(1),Z(2),Z(3),0.5,'b');

% Plot WORLD reference system
quiver3(0,0,0, 1,0,0,0.5,'r', 'LineWidth',2);
quiver3(0,0,0, 0,1,0,0.5,'g', 'LineWidth',2);
quiver3(0,0,0, 0,0,1,0.5,'b', 'LineWidth',2);

legend('X_{cam}', 'Y_{cam}', 'Z_{cam}', 'X_{world}', 'Y_{world}', 'Z_{world}', ...
      'Location','northeastoutside')

xlabel('X_{world}')
ylabel('Y_{world}')
zlabel('Z_{world}')

% Turn axis as world reference system
view(-30,45)
grid on
ax = gca;
ax.XDir = "reverse";
ax.ZDir = "reverse";

% --------------- PLOT WORLD POINTS QUADRANGLE ---------------
world_points = [UL_new_cam, UR_new_cam, BR_new_cam, BL_new_cam];
plotResults_p1(world_points, R_cam_to_world, false);

% --------------- PLOT PARALLELEPIPED CAR ---------------
plotResults_p1(world_points, R_cam_to_world, true);