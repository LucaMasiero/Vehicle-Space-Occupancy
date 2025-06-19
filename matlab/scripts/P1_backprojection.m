%% Retrieve points from image

addpath("matlab\functions")
close all
clear

% I-PHONE
%image_path = "/imgs/pandina/iPhone/panda.jpg";
load("./matlab/data/iPhone_camera_params.mat") 

% NOTHING PHONE 2a
%image_path = "/imgs/ibiza/nothing2a/seat_1.jpg";
%load("./matlab/data/nothing2a_camera_params_LR.mat")

[file,location] = uigetfile({'*jpg'; '*png'});
if isequal(file,0)
   disp('User selected Cancel');
else
   abs_path = fullfile(location,file);
   disp(['User selected ', abs_path]);
end

K = cameraParams.Intrinsics.K;                              % camera calibration
[x,y] = selectFeatures(abs_path, cameraParams.Intrinsics);  % feature (image) points
%% Compute world points position in camera coordinates
UL = [x(1);y(1);1]; 
UR = [x(2);y(2);1];
BR = [x(3);y(3);1];
BL = [x(4);y(4);1];

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

% Pandina: taillights distance=114.5/122cm; license plate=53cm
% Ibiza: taillights distance=104cm; license plate=53
[dUL, dUR] = deriveDistance(UL_dir, UR_dir, inf_dir, 114.5);
[dBL, dBR] = deriveDistance(BL_dir, BR_dir, inf_dir, 53);

% Coordinates of the real world points in the camera ref. system
UL_cam = dUL*UL_dir;
UR_cam = dUR*UR_dir;
BL_cam = dBL*BL_dir;
BR_cam = dBR*BR_dir;

%% Derive camera rotation matrix
X_axis = inf_dir;

% Find another axis perpendicular to X_axis
temp = [0, 1, 0]';  % Arbitrary vector
if abs(dot(X_axis, temp)) > 0.9     % if too parallel
    temp = [0, 0, 1]';
end
    
Y_axis = cross(temp, X_axis);
Y_axis = Y_axis / norm(Y_axis);

% Z-axis completes the coordinate system
Z_axis = cross(X_axis, Y_axis);

% Step 2: Form the rotation matrix (reference system of the world w.r.t. camera reference system)
R_world_to_cam = [X_axis, Y_axis, Z_axis];
% reference system of the camera w.r.t. world reference system
R_cam_to_world = R_world_to_cam';

% Rotate points as the world reference frame to obtain a result 
% which is more similar to what we would expect. Remember that the camera
% Z-axis is pointing towards the principal point
UL_new_cam = R_cam_to_world*UL_cam;
UR_new_cam = R_cam_to_world*UR_cam;
BL_new_cam = R_cam_to_world*BL_cam;
BR_new_cam = R_cam_to_world*BR_cam;

%% Plot camera and world points (car)

world_points = [UL_new_cam, UR_new_cam, BR_new_cam, BL_new_cam];
x = world_points(1,:);
y = world_points(2,:);
z = world_points(3,:);

X = R_cam_to_world(:,1);
Y = R_cam_to_world(:,2);
Z = R_cam_to_world(:,3);

figure()
scatter3(x,y,z, 'bo');  % world points
fill3(x,y,z,'b')
hold on
scatter3(0,0,0, 'r+');  % camera center

quiver3(0,0,0, X(1),X(2),X(3),100,'r')
quiver3(0,0,0, Y(1),Y(2),Y(3),100,'g')
quiver3(0,0,0, Z(1),Z(2),Z(3),100,'b')

pose = rigid3d(R_cam_to_world',[0,0,0]);
plotCamera("AbsolutePose",pose,"Size",15)

title("Camera Reference System")

xlabel('X')
ylabel('Y')
zlabel('Z')
grid on

% turn axis as world reference system
ax = gca;
ax.XDir = "reverse";
ax.ZDir = "reverse";

%% Plot of the parallelepiped simplifying the shape of the car
world_points = [UL_new_cam, UR_new_cam, BR_new_cam, BL_new_cam];
x = world_points(1,:);
y = world_points(2,:);
z = world_points(3,:);

X = R_cam_to_world(:,1);
Y = R_cam_to_world(:,2);
Z = R_cam_to_world(:,3);

% Parallelepiped vertices
P1 = BL_new_cam + [52.5,-15,75]';
P2 = P1 + [-157.8,0,0]';
P3 = P1 + [0,0,-157.8]';
P4 = P2 + [0,0,-157.8]';
P5 = P1 + [0,353.8,0]';
P6 = P2 + [0,353.8,0]';
P7 = P3 + [0,353.8,0]';
P8 = P4 + [0,353.8,0]';

back_side = [P1,P2,P4,P3]';
front_side = [P5,P6,P8,P7]';
left_side = [P1,P3,P7,P5]';
right_side = [P2,P6,P8,P4]';
top_side = [P3,P4,P8,P7]';
bottom_side = [P1,P2,P6,P5]';
xPatch= [back_side(:,1), front_side(:,1), left_side(:,1), right_side(:,1), top_side(:,1), bottom_side(:,1)];
yPatch = [back_side(:,2), front_side(:,2), left_side(:,2), right_side(:,2), top_side(:,2), bottom_side(:,2)];
zPatch = [back_side(:,3), front_side(:,3), left_side(:,3), right_side(:,3), top_side(:,3), bottom_side(:,3)];

figure()
scatter3(x,y,z, 'bo');  % world points
hold on
patch(xPatch,yPatch,zPatch,'c','FaceAlpha',.5);
fill3(x,y,z,'b')
scatter3(0,0,0, 'r+');  % camera center

quiver3(0,0,0, X(1),X(2),X(3),100,'r')
quiver3(0,0,0, Y(1),Y(2),Y(3),100,'g')
quiver3(0,0,0, Z(1),Z(2),Z(3),100,'b')

pose = rigid3d(R_cam_to_world',[0,0,0]);
plotCamera("AbsolutePose",pose,"Size",15)

title("Camera Reference System")

xlabel('X')
ylabel('Y')
zlabel('Z')
grid on
axis equal

xlim([-100,300])
ylim([-20,700])

% turn axis as world reference system
ax = gca;
ax.XDir = "reverse";
ax.ZDir = "reverse";

%% Plot camera reference system w.r.t. world reference system
% We applied R_cam_to_world rotation to move the points in a camera
% reference system that is alligned with the world reference system
% (ideally aligned with the street)

% Take camera reference axis
X = R_cam_to_world(:,1);
Y = R_cam_to_world(:,2);
Z = R_cam_to_world(:,3);

figure()
quiver3(0,0,0, X(1),X(2),X(3),0.5,'r')
hold on
quiver3(0,0,0, Y(1),Y(2),Y(3),0.5,'g')
quiver3(0,0,0, Z(1),Z(2),Z(3),0.5,'b')

% Plot world reference axis
quiver3(0,0,0, 1,0,0,0.5,'y')
quiver3(0,0,0, 0,1,0,0.5,'m')
quiver3(0,0,0, 0,0,1,0.5,'c')

xlabel('X')
ylabel('Y')
zlabel('Z')

ax = gca;
ax.XDir = "reverse";
ax.ZDir = "reverse";
