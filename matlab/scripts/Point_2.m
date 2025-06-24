addpath("matlab\functions")

close all
clear

% I-PHONE
%image_path = "/imgs/pandina/iPhone/panda.jpg";
load("./matlab/data/iPhone_camera_params.mat")

% NOTHING PHONE 2a
%image_path = "/imgs/ibiza/nothing2a/seat_1.jpg";
%load("./matlab/data/nothing2a_camera_params_LR.mat")

K = cameraParams.Intrinsics.K;

% Extract feature points in each frame
[file,location] = uigetfile({'*.*'; '*jpg'}, 'Folder', what('imgs').path, 'MultiSelect', 'on');
if isequal(file,0)
   disp('User Canceled Selection');
end

feature_pts = {};
for i=1:3
    abs_path = fullfile(location,file(i));
    feature_pts{i} = selectFeatures_p2(abs_path{1}, cameraParams.Intrinsics);
    disp(['Frame ', num2str(i),': features selected'])
end

%% Define and solve system to compute world reference system
p_prime = zeros(3,2);    % init p_prime
q_prime = zeros(3,2);   % init q_prime

for k=1:3
    p_prime(:,k)= [feature_pts{k}(:,1); 1];
    q_prime(:,k) = [feature_pts{k}(:,2); 1];
end

function err = compute_distance_error(a, b, p_prime, q_prime, K)
    n = [a 1 b]'/sqrt(a^2 + b^2 + 1);
    i = [1 -a 0]'/sqrt(a^2 + 1);
    j = [a*b -b a^2+1]'/( sqrt(a^2 + b^2 + 1)*sqrt(a^2 + 1) );

    N = size(p_prime, 2);
    dists = zeros(1,N);

    for k = 1:N
        p_k = [i'; j']*(inv(K)*p_prime(:,k)) / (n'*inv(K)*p_prime(:,k));
        q_k = [i'; j']*(inv(K)*q_prime(:,k)) / (n'*inv(K)*q_prime(:,k));
        dists(k) = norm(p_k - q_k);
    end

    d_mean = mean(dists);
    err = sum((dists - d_mean).^2);
end

% Define error function
error_fun = @(x) compute_distance_error(x(1), x(2), p_prime, q_prime, K);
% Initial guess for a, b
x0 = [0, 0];
% Use nonlinear least squares optimizer
x_opt = fminsearch(error_fun, x0);
a = x_opt(1)
b = x_opt(2)

% Copmute 'optimal' axes
n = [a 1 b]'/sqrt(a^2 + b^2 + 1);
i = [1 -a 0]'/sqrt(a^2 + 1);
j = [a*b -b a^2+1]'/( sqrt(a^2 + b^2 + 1)*sqrt(a^2 + 1) );  
%% Compute car position in each frame
% Real taillight positions in car coordinate frame centered in the middle
% point of pq segment
w_real = 1.145; % [m]
car_pts = [
    -w_real/2, 0, 0;  % Left taillight
     w_real/2, 0, 0   % Right taillight
]'; 

cam_pts = {};
for f=1:3
    % Backproject image points and compute distance from camera
    p3D = inv(K) * p_prime(:,f) / abs(n' * inv(K) * p_prime(:,f));
    q3D = inv(K) * q_prime(:,f) / abs(n' * inv(K) * q_prime(:,f));
    
    % Compute scale factor
    w_estimate = norm(p3D - q3D);
    s = w_real/w_estimate;
    
    cam_pts{f} = [s*p3D, s*q3D];   % 3x2 matrix
    
    % Use Kabsch algorithm for image registration; (you can also use
    % Procrustes analysis)
    [U, ~, V] = svd(cam_pts{f} * car_pts');     % compute svd of the covariance matrix
    d = det(U)*det(V);                          % check if the orthogonal matrices contain a reflection
    R = U * diag([1,1,d]) * V';                 % calculate optimal rotation matrix R
    
    % Derive position of middle point between p and q in camera coordinates
    t(:,f) = mean(cam_pts{f}, 2) - R * mean(car_pts, 2);
end
t

%% World and Camera reference systems comparison
% Take camera reference axis
Z_inv = [1,0,0; 0,1,0; 0,0,-1];
X = R(:,1);
Y = R(:,3);
Z = R(:,2);

figure()
title("World and Camera reference systems comparison")
hold on
grid on
% Plot camera reference axes in world frame
quiver3(0,0,0, X(1),X(2),X(3), 0.5,'r')
quiver3(0,0,0, Y(1),Y(2),-Y(3), 0.5,'g')
quiver3(0,0,0, Z(1),Z(2),Z(3), 0.5,'b')

% Plot world reference axes
quiver3(0, 0, 0, 1,0,0, 0.5, 'r', 'LineWidth', 2);
quiver3(0, 0, 0, 0,1,0, 0.5, 'g', 'LineWidth', 2);
quiver3(0, 0, 0, 0,0,1, 0.5, 'b', 'LineWidth', 2);

legend('X_{cam}', 'Y_{cam}', 'Z_{cam}', 'X_{world}', 'Y_{world}', 'Z_{world}')

xlabel('X')
ylabel('Y')
zlabel('Z')
lim = 0.6;
xlim([-0.2,lim])
ylim([-0.2,lim])
zlim([-lim,lim])

view(35,30)

%% Plot Results :)
figure
title("Results")
hold on
grid on

% Plot parallelepiped (=car) for each frame
for f=1:3
    p = cam_pts{f}(:,1);
    p = [p(1);
         p(3);
         p(2)];
    q = cam_pts{f}(:,2);
    q = [q(1);
         q(3);
         q(2)];

    plotResults_P2(p,q);
end

% Plot camera
% rotate R 90 degrees to account for Y-Z switch
permutation = [1 0 0;   
               0 0 -1;
               0 1 0];
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
