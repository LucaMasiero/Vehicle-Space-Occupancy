%%
%close all
clear
%%
pen_tests = true;
use_DLT = false;
rotate_frame = false;
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

m = {};
for i=1:3
    abs_path = fullfile(location,file(i));
    [p_prime(:,i), q_prime(:,i), m{i}] = selectFeatures_p2(abs_path{1}, cameraParams.Intrinsics, use_DLT);
    disp(['Frame ', num2str(i),': features selected'])
end

p_prime(2,:) = cameraParams.Intrinsics.ImageSize(1) - p_prime(2,:);
q_prime(2,:) = cameraParams.Intrinsics.ImageSize(1) - q_prime(2,:);

p_prime_orig = p_prime;
q_prime_orig = q_prime;

%% Apply DLT to each frame
if use_DLT
    m2 = m{1};
    for k=1:3
        m1 = m{k};
        H = DLT(m1, m2);
        Hm = H*m{k};
        Hp = H*p_prime(:,k);
        Hq = H*q_prime(:,k);
        m{k} = Hm./repmat(Hm(3,:),3,1);
        Hp_prime(:,k) = Hp./repmat(Hp(3,:),3,1);
        Hq_prime(:,k) = Hq./repmat(Hq(3,:),3,1);
    end
    
    %t = maketform( 'projective', H');
    %J = imtransform(img3,t);

    figure;
    abs_path = fullfile(location,file(1));
    img1 = imread(abs_path{1});
    imshow(img1);
    hold on
    MRK_SZ = 20;
    scatter(p_prime(1,1),p_prime(2,1),MRK_SZ,'ro','filled');
    scatter(p_prime(1,2),p_prime(2,2),MRK_SZ,'mo','filled');
    scatter(p_prime(1,3),p_prime(2,3),MRK_SZ,'bo','filled');
    scatter(q_prime(1,1),q_prime(2,1),MRK_SZ,'ro','filled');
    scatter(q_prime(1,2),q_prime(2,2),MRK_SZ,'mo','filled');
    scatter(q_prime(1,3),q_prime(2,3),MRK_SZ,'bo','filled');

    scatter(Hp_prime(1,1),Hp_prime(2,1),MRK_SZ,'ro');
    scatter(Hp_prime(1,2),Hp_prime(2,2),MRK_SZ,'mo');
    scatter(Hp_prime(1,3),Hp_prime(2,3),MRK_SZ,'bo');
    scatter(Hq_prime(1,1),Hq_prime(2,1),MRK_SZ,'ro');
    scatter(Hq_prime(1,2),Hq_prime(2,2),MRK_SZ,'mo');
    scatter(Hq_prime(1,3),Hq_prime(2,3),MRK_SZ,'bo');
end

%%
p_prime = p_prime_orig;
q_prime = q_prime_orig;
%p_prime = Hp_prime;
%q_prime = Hq_prime;

%% Define and solve system to compute world reference system
function err = compute_distance_error(a, b, c, p_prime, q_prime, K)
    n = [a b c]'/sqrt(a^2 + b^2 + c^2);
    %i = [1 -a 0]'/sqrt(a^2 + 1);
    %j = [a*b -b a^2+1]'/( sqrt(a^2 + b^2 + 1)*sqrt(a^2 + 1) );

    N = size(p_prime, 2);
    dists = zeros(1,N);

    for k = 1:N
        dp = inv(K)*p_prime(:,k);   % back-project ray
        dp = dp / norm(dp);         % normalize ray
        p_k2 = dp / dot(n,dp);      % intersect with plane with normal vector n
        dq = inv(K)*q_prime(:,k);
        dq = dq / norm(dq);
        q_k2 = dq / dot(n,dq);

        dists(k) = norm(p_k2 - q_k2);
    end

    d_real = 1.145;
    err = sum((dists - d_real).^2);
end

% Define error function
error_fun = @(x) compute_distance_error(x(1), x(2), x(3), p_prime, q_prime, K);
% Initial guess for a, b
x0 = [0, 0, 1];
% Use nonlinear least squares optimizer
x_opt = fminsearch(error_fun, x0, optimset('MaxFunEvals',1000));
a = x_opt(1)
b = x_opt(2)
c = x_opt(3)

% Copmute 'optimal' axes
n = [a b c]'/sqrt(a^2 + b^2 + c^2);
%i = [1 -a 0]'/sqrt(a^2 + 1);
%j = [a*b -b a^2+1]'/( sqrt(a^2 + b^2 + 1)*sqrt(a^2 + 1) );

dp = inv(K)*p_prime(:,1);
dp = dp / norm(dp);
%if dot(n,dp) < 0
%    n = -n;
%end
%% Compute car position in each frame
% Real taillight positions in car coordinate frame centered in the middle
% point of pq segment
if pen_tests
    w_real = 0.15;
else
    w_real = 1.145; % [m]
end
car_pts = [
    -w_real/2, 0, 0;  % Left taillight
     w_real/2, 0, 0   % Right taillight
]'; 

cam_pts = {};
for k=1:3
    % Backproject image points and compute distance from camera
    pd = inv(K) * p_prime(:,k);
    pd = pd / norm(pd);
    p3D = pd / dot(n,pd);
    qd = inv(K)*q_prime(:,k);
    qd = qd / norm(qd);
    q3D = qd / dot(n,qd);
    
    % Compute scale factor
    %w_estimate = norm(p3D - q3D);
    %s = w_real/w_estimate;
    
    cam_pts{k} = [p3D, q3D];   % 3x2 matrix
end

%% World and Camera reference systems comparison
R = [i,j,n];    % Plane π axes w.r.t. camera reference frame 
%R = RR;        % matrix describing the straightened frame w.r.t. camera frame

% visualization: invert y and z coordinates to account for difference
% between matlab and camera reference frames
tmp = R(2,:);
R(2,:) = R(3,:);
R(3,:) = tmp;

X = R(:,1);
Y = R(:,2);
Z = R(:,3);

figure()
title("World and Camera reference systems comparison")
hold on
grid on
% Plot word reference frame in camera coordinate
quiver3(0,0,0, X(1),X(2),X(3), 0.5,'r')
quiver3(0,0,0, Y(1),Y(2),Y(3), 0.5,'g')
quiver3(0,0,0, Z(1),Z(2),Z(3), 0.5,'b')

% Plot camera reference axes
quiver3(0, 0, 0, 1,0,0, 0.5, 'r', 'LineWidth', 2);
quiver3(0, 0, 0, 0,0,1, 0.5, 'g', 'LineWidth', 2);
quiver3(0, 0, 0, 0,1,0, 0.5, 'b', 'LineWidth', 2);

legend('X_{π}', 'Y_{π}', 'Z_{π}', 'X_{cam}', 'Y_{cam}', 'Z_{cam}')

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

%% Rotate camera reference frame
if rotate_frame
    % correct points
    points = [cam_pts{1}, cam_pts{2}, cam_pts{3}];
    RR = findRotationToZ(points');
    % Rotate points
    rotated_points = (RR * points);
    
    % Visualization: matlab Y and Z are swapped to represent camera frame
    %tmp = rotated_points(2,:);
    %rotated_points(2,:) = rotated_points(3,:);
    %rotated_points(3,:) = tmp;
    
    new_cam_pts = {};
    new_cam_pts{1} = rotated_points(:,1:2);
    new_cam_pts{2} = rotated_points(:,3:4);
    new_cam_pts{3} = rotated_points(:,5:6);

else
    % keep the same points
    new_cam_pts = cam_pts;
end
%% Plot Results: Parallelepiped
figure
title("Results")
hold on
grid on

% Plot parallelepiped (=car) for each frame
colors = ['r', 'm', 'b'];
for k=1:3
    p = new_cam_pts{k}(:,1);
    p = [p(1);
         p(3);
         p(2)];
    q = new_cam_pts{k}(:,2);
    q = [q(1);
         q(3);
         q(2)];

    plotResults_P2(p,q,true,colors(k));
end

% Plot camera
% rotate R 90 degrees to account for Y-Z switch
permutation = [1 0 0;   
               0 0 -1;
               0 1 0];
% To translate up the camera
%max_height = max(rotated_points(2,:));
%pose = rigid3d(permutation,[0,0,max_height-1.7]);
pose = rigid3d(permutation,[0,0,0]);
plotCamera("AbsolutePose",pose,"Size",0.5)

xlabel('X');
ylabel('Z');    % since we have swaped Y with Z ...
zlabel('Y');    % ... and Z with Y

axis equal
ax = gca;
ax.ZDir = "reverse";    % reversing matlab Z-axis,
                        % which for us is the Y-axis (vertical)
view(45, 45);

%% Plot Results: just feature points and segment - for tests with PEN
figure
title("Results")
hold on
grid on

% Plot parallelepiped (=car) for each frame
colors = ['r', 'm', 'b'];
for k=1:3
    p = new_cam_pts{k}(:,1);
    p = 100*[p(1);
         p(2);
         p(3)];
    q = new_cam_pts{k}(:,2);
    q = 100*[q(1);
         q(2);
         q(3)];

    plotResults_P2(p,q,false,colors(k));
end

% Plot camera
% rotate R 90 degrees to account for Y-Z switch
permutation = [1 0 0;   
               0 0 -1;
               0 1 0];
% To translate up the camera
if rotate_frame
    max_height = max(rotated_points(2,:));
    pose = rigid3d(permutation,[0,0,100*(max_height-0.1)]);
else
    pose = rigid3d(permutation,[0,0,0]);
end
%plotCamera("AbsolutePose",pose,"Size",5)

xlabel('X');
ylabel('Z');    % since we have swaped Y with Z ...
zlabel('Y');    % ... and Z with Y

axis equal
ax = gca;
ax.ZDir = "reverse";    % reversing matlab Z-axis,
                        % which for us is the Y-axis (vertical)
view(45, 45);
%%
function R = findRotationToZ(points)

    % 1. Compute the centroid of the points
    centroid = mean(points, 1);

    % 2. Subtract centroid to center the data
    centeredPoints = points - centroid;
    
    % 3. Perform SVD
    [~, ~, V] = svd(centeredPoints, 'econ');
    
    % 4. The normal vector is the third column of V (least variance direction)
    normal = V(:,3);
    if normal(2) > 0
        normal = -normal; % we want an upward z axis
    end
    
    % 5. Desired normal direction (Z-axis)
    zAxis = [0; 0; 1];
    
    % 6. Compute rotation matrix to align normal -> zAxis
    R = rotationMatrixFromVectors(normal, zAxis);
    
end

function R = rotationMatrixFromVectors(a, b)
% ROTATIONMATRIXFROMVECTORS finds the rotation matrix that rotates vector a to b

    a = a / norm(a);
    b = b / norm(b);
    v = cross(a, b);
    c = dot(a, b);
    
    if abs(c - 1) < 1e-10
        R = eye(3); % Already aligned
        return;
    elseif abs(c + 1) < 1e-10
        % Choose an arbitrary orthogonal vector
        axis = null(a');  % returns two orthonormal vectors
        v = axis(:,1);    % pick one
        H = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
        R = eye(3) + 2*H*H;  % special case for 180 deg
        return;
    end
    
    vx = [    0   -v(3)  v(2);
           v(3)     0  -v(1);
          -v(2)  v(1)     0 ];
      
    R = eye(3) + vx + vx^2 * ((1 - c) / norm(v)^2);
end
