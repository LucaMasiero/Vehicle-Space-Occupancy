close all
clear

%%
PEN_LENGTH = 0.15;    %[m]
CAR_LENGTH = 1.145;   %[m]
COLORS = ['r', 'm', 'b'];
MRK_SZ = 20;

PenTest = false;
DLTon = false;
RotateToPiFrame = true;
%%
addpath("matlab\functions")

% I-PHONE
%image_path = "/imgs/pandina/iPhone/panda.jpg";
load("./matlab/data/iPhone_camera_params.mat")

% NOTHING PHONE 2a
%image_path = "/imgs/ibiza/nothing2a/seat_1.jpg";
%load("./matlab/data/nothing2a_camera_params_HR.mat")

K = cameraParams.Intrinsics.K;


% Extract feature points in each frame
[file,location] = uigetfile({'*.*'; '*jpg'}, 'Folder', what('imgs').path, 'MultiSelect', 'on');
if isequal(file,0)
   disp('User Canceled Selection');
end

image_points = zeros(3, 2, 3);
m = {}; % features for DLT
for i=1:3
    abs_path = fullfile(location,file(i));
    [image_points(i,1,:), image_points(i,2,:), m{i}] = selectFeatures_p2(abs_path{1}, cameraParams.Intrinsics, DLTon);
    disp(['Frame ', num2str(i),': features selected'])
end

%%
% Apply DLT to each frame
HImage_points = zeros(3,2,3);
if DLTon
    for i=1:3
        % Compute homography from frame k (m{i}) to the first frame (m{1})
        H = DLT(m{i}, m{1});
        % Transform DLT features
        Hm = H*m{i};
        m{i} = Hm./repmat(Hm(3,:),3,1);
        % Transform car features (p and q)
        Hp = H * squeeze(image_points(i,1,:));
        Hq = H * squeeze(image_points(i,2,:));
        HImage_points(i,1,:) = Hp./Hp(3);
        HImage_points(i,2,:) = Hq./Hq(3);
    end

    figure;
    abs_path = fullfile(location,file(1));
    img1 = imread(abs_path{1});
    imshow(img1);
    hold on
    for i=1:3
        % Plot extracted points
        scatter(image_points(i,1,1),image_points(i,1,2),MRK_SZ,'o', 'MarkerEdgeColor',COLORS(i));
        scatter(image_points(i,2,1),image_points(i,2,2),MRK_SZ,'o', 'MarkerEdgeColor',COLORS(i));

        % Plot transformed points
        scatter(HImage_points(i,1,1),HImage_points(i,1,2),MRK_SZ,'o','filled', 'MarkerEdgeColor',COLORS(i), 'MarkerFaceColor',COLORS(i));
        scatter(HImage_points(i,2,1),HImage_points(i,2,2),MRK_SZ,'o','filled', 'MarkerEdgeColor',COLORS(i), 'MarkerFaceColor',COLORS(i));
    end

    % Substitute with transformed features
    image_points = HImage_points;
end

% --------------- MAIN METHOD ---------------
if PenTest
    d = PEN_LENGTH;
else
    d = CAR_LENGTH;
end
[P_list, Q_list, normal] = localize_car_points(K, image_points, d, cameraParams.Intrinsics.ImageSize, PenTest);

% --------------- ALIGN CAMERA Z AXIS TO NORMAL VECTOR TO PLANE π ---------------
if RotateToPiFrame
    R = rotationMatrixFromVectors(normal, [0;1;0]);

    % Rotate points
    for i=1:3
        P_list(i,:) = (R*P_list(i,:)')';
        Q_list(i,:) = (R*Q_list(i,:)')';
    end

    % Visualization: matlab Y and Z are swapped to represent camera frame
    P_list(:,[2,3]) = P_list(:,[3,2]);
    Q_list(:,[2,3]) = Q_list(:,[3,2]);
end
%% --------------- PLOT RESULTS: PARALLELEPIPED ---------------
figure
title("Car trajectory")
hold on
grid on

% Plot parallelepiped (=car) for each frame
for i=1:3
    p = P_list(i,:);
    p = [p(1);
         p(2);
         p(3)];
    q = Q_list(i,:);
    q = [q(1);
         q(2);
         q(3)];

    plotResults_P2(p,q,true,COLORS(i));
end

% Plot camera
if ~exist('R', 'var')
    % rotate R 90 degrees to account for Y-Z switch
    R = [1 0 0;   
         0 0 1;
         0 -1 0];
end
pose = rigid3d(R*[1 0 0; 0 0 -1; 0 1 0] , [0,0,0]);
plotCamera("AbsolutePose",pose,"Size",0.5)

xlabel('X');
ylabel('Z');    % since we have swaped Y with Z ...
zlabel('Y');    % ... and Z with Y

axis equal
ax = gca;
ax.ZDir = "reverse";    % reversing matlab Z-axis,
                        % which for us is the Y-axis (vertical)
view(45, 45);

%% --------------- NORMAL VECTOR n W.R.T. CAMERA REFERENCE FRAME --------------
figure()
title("Normal to plane π")
hold on
grid on
% Plot normal vector to plane
quiver3(0,0,0, normal(1),normal(3),normal(2), 0.5,'b')

% Plot camera reference axes
quiver3(0, 0, 0, 1,0,0, 0.5, 'r', 'LineWidth', 2);
quiver3(0, 0, 0, 0,0,1, 0.5, 'g', 'LineWidth', 2);
quiver3(0, 0, 0, 0,1,0, 0.5, 'b', 'LineWidth', 2);

legend('n_π', 'X_{cam}', 'Y_{cam}', 'Z_{cam}')

xlabel('X')
ylabel('Z')
zlabel('Y')
lim = 0.6;
xlim([-0.2,lim])
ylim([-0.2,lim])
zlim([-lim,lim])

ax = gca;
ax.ZDir = 'reverse';

view(-35,30)

%% Plot Results: just feature points and segment - for tests with PEN
figure
title("PenTest Results")
hold on
grid on

% Plot parallelepiped (=car) for each frame
for i=1:3
    p = P_list(i,:);
    p = 10*[p(1);
             p(2);
             p(3)];
    q = Q_list(i,:);
    q = 10*[q(1);
             q(2);
             q(3)];
    
    plotResults_P2(p,q, false, COLORS(i));
end

% Plot camera
if ~exist('R', 'var')
    % rotate R 90 degrees to account for Y-Z switch
    R = [1 0 0;   
         0 0 1;
         0 -1 0];
end
% To translate up the camera
pose = rigid3d(R*[1 0 0; 0 0 -1; 0 1 0], [0,0,0]);
plotCamera("AbsolutePose",pose,"Size",1)

xlabel('X');
ylabel('Z');    % since we have swaped Y with Z ...
zlabel('Y');    % ... and Z with Y

axis equal
ax = gca;
ax.ZDir = "reverse";    % reversing matlab Z-axis,
                        % which for us is the Y-axis (vertical)
view(45, 45);
