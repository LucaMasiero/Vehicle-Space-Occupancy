close all
clear
addpath("matlab\functions")

PEN_LENGTH = 0.15;    %[m]
CAR_LENGTH = 1.145;   %[m]
COLORS = ['r', 'm', 'b'];
MRK_SZ = 20;

% Here you can choose whether:
%   - to use pen or car frames,
%   - to use DLT or not,
%   - to save the extracted features,
%   - to use 'precooked' features
%
PenTest = false;
DLTon = false;
saveFeatures = false;
UseExamples = false;

if PenTest
    d = PEN_LENGTH;
else
    d = CAR_LENGTH;
end

% --------------- FEATURE EXTRACTION ---------------
% ATTENTION: it is very important the the features on the car are selected
% in the following order:
%   - p, which is on the left taillight
%   - q, which is on the right taillight
%   - DLT features if needed (four points on the background)
%

% Choose the camera
% I-PHONE
load("./matlab/data/cameras/iPhone_camera_params.mat")
K = cameraParams.Intrinsics.K;

if UseExamples
    % Select precooked.mat file from the desired sequence
    [file,location] = uigetfile({'*.*'; '*.mat'}, 'Folder', what('imgs\pandina\iPhone\').path);
    abs_path = fullfile(location,file);
    % Retrieve car and DLT feature points
    feature_points = load(abs_path);
    image_points = feature_points.image_points;
    m =  feature_points.m;
    disp('Extracted features from precooked file.')

    % Check if DLT is on but no features were saved for DLT
    if isempty(m) && DLTon
        DLTon = false;
        disp("DLT was disabled since there is no saved DLT features in "+file);
    end
    % For precooked features always the first three features were considered
    file = ["1.jpg", "2.jpg", "3.jpg"];

    % Plot features on frames
    for i=1:3
        imgPath = fullfile(location, file(i));
        plotFeaturesOnImage_p2(imgPath, squeeze(image_points(i,1,:)), ...
                                        squeeze(image_points(i,2,:)), ...
                                        m{i}, ...
                                        cameraParams.Intrinsics. ...
                                        DLTon, false);
    end
else
    % Select frames
    [file,location] = uigetfile({'*.*'; '*jpg'}, 'Folder', what('imgs\pandina\iPhone\').path, 'MultiSelect', 'on');
    if isequal(file,0)
       disp('User Canceled Selection');
    end
    
    % Extract features from frames
    image_points = zeros(3, 2, 3);
    m = {}; % features for DLT
    for i=1:3
        abs_path = fullfile(location,file(i));
        [image_points(i,1,:), image_points(i,2,:), m{i}] = selectFeatures_p2(abs_path{1}, cameraParams.Intrinsics, DLTon);
        disp(['Frame ', num2str(i),': features selected'])
    end

    if saveFeatures
        % Save features if required
        save(location+"\precooked.mat", "image_points", "m");
    end
end

% --------------- APPLY DLT IF REQUIRED ---------------
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
    [img1, ~] = imresize(img1, cameraParams.Intrinsics.ImageSize);
    imshow(rgb2gray(img1));
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
[P_list, Q_list, normal] = localize_car(K, image_points, d);

% --------------- ALIGN CAMERA Y AXIS TO NORMAL VECTOR TO PLANE π ---------------
R = rotationMatrixFromVectors(normal, [0;1;0]);

% Rotate points
for i=1:3
    P_list(i,:) = (R*P_list(i,:)')';
    Q_list(i,:) = (R*Q_list(i,:)')';
end

% Visualization: matlab Y and Z are swapped to represent camera frame
P_list(:,[2,3]) = P_list(:,[3,2]);
Q_list(:,[2,3]) = Q_list(:,[3,2]);

% --------------- PLOT NORMAL VECTOR n w.r.t. CAMERA REFERENCE FRAME --------------
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

% --------------- PLOT TRAJECTORY RESULTS: CAR or PEN---------------
if PenTest
    % For plotting pen trajectory (i.e., test)
    ttl = "Pen trajectory";
    drawParallelepiped = false;
    cameraSize = 0.08;
else
    % For plotting car trajectory 
    ttl = "Car trajectory";
    drawParallelepiped = true;
    cameraSize = 0.5;
end

figure
title(ttl)
hold on
grid on
for i=1:3
    p = P_list(i,:);
    p = [p(1);
         p(2);
         p(3)];
    q = Q_list(i,:);
    q = [q(1);
         q(2);
         q(3)];

    plotResults_p2(p,q,drawParallelepiped,COLORS(i));
end

% Plot camera
pose = rigid3d(R*[1 0 0; 0 0 -1; 0 1 0] , [0,0,0]);
plotCamera("AbsolutePose",pose,"Size",cameraSize)

xlabel('X');
ylabel('Z');    % since we have swaped Y with Z ...
zlabel('Y');    % ... and Z with Y

axis equal
ax = gca;
ax.ZDir = "reverse";    % reversing matlab Z-axis,
                        % which for us is the Y-axis (vertical)
view(45, 45);
