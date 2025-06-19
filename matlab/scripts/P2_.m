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

%%
syms a b

n = [a 1 b]'/sqrt(a*a + b*b + 1);
i = [1 -a 0]'/sqrt(a*a + 1);
j = [a*b -b a^2+1]'/( sqrt(a^2 + a^2 + 1)*sqrt(a^2 + 1) );  % from slides
%j = [-a*b -b a^2+1]'/(sqrt(a^2*b^2 + b^2 + a^4 + 1 + 2*a^2));      % our version

p = sym(zeros(2,3));
q = sym(zeros(2,3));
for k=1:3
    p_prime(:,k)= [feature_pts{k}(1,:), 1]';
    q_prime(:,k) = [feature_pts{k}(2,:), 1]';

    p(:,k) = [i'; j']*(inv(K)*p_prime(:,k))/(n'*inv(K)*p_prime(:,k)); % back-projection and projection on plane π
    q(:,k) = [i'; j']*(inv(K)*q_prime(:,k))/(n'*inv(K)*q_prime(:,k));
end

% Define system:
% the distance between p and q on plane π must always be the same
eqn1 = norm(p(:,1)-q(:,1)) == norm(p(:,2)-q(:,2));
eqn2 = norm(p(:,3)-q(:,3)) == norm(p(:,2)-q(:,2));
eqns = [eqn1,eqn2];

% Solve system
solutions = solve(eqns, [a b])
a = double(solutions.a);
b = double(solutions.b);

n = [a 1 b]'/sqrt(a^2 + b^2 + 1);
i = [1 -a 0]'/sqrt(a^2 + 1);
%j = [-a*b -b a^2+1]'/(sqrt(a^2*b^2 + b^2 + a^4 + 1 + 2*a^2));      % our version
j = [a*b -b a^2+1]'/( sqrt(a^2 + a^2 + 1)*sqrt(a^2 + 1) );  % from slides
%% Compute car position in each frame
% Real taillight positions in car coordinate frame
w_real = 1.145; % [m]
car_pts = [
    -w_real/2, 0, 0;  % Left taillight
     w_real/2, 0, 0   % Right taillight
]';

cam_pts = {};
for f=1:3
    % Backproject image points
    p3D = inv(K) * p_prime(:,f) / (n' * inv(K) * p_prime(:,f));
    q3D = inv(K) * q_prime(:,f) / (n' * inv(K) * q_prime(:,f));
    
    w_estimate = norm(p3D - q3D);
    s = w_real/w_estimate;
    
    cam_pts{f} = [s*p3D, s*q3D];   % 3x2 matrix
    
    % Use Kabsch algorithm for image registration; (you can also use
    % Procrustes analysis)
    [U, ~, V] = svd(cam_pts{f} * car_pts');    % compute svd of the covariance matrix
    d = det(U)*det(V);                      % check if the orthogonal matrices contain a reflection
    R = U * diag([1,1,d]) * V';             % calculate optimal rotation matrix R
    
    % derive position of world origin
    t(:,f) = mean(cam_pts{f}, 2) - R * mean(car_pts, 2); %???
end
t
%% Plot

% Define the normal vector
normal = n;

% Choose a point on the plane (any point will do)
point = p3D;

% Create a grid for x and y coordinates
x = linspace(-5, 5, 30);
y = linspace(-5, 5, 30);
[X, Y] = meshgrid(x, y);

% Calculate z coordinates using the plane equation
% First check if the z component is close to zero
%if abs(normal(3)) < 1e-10
%    error('The z-component of the normal vector is too close to zero. The plane is vertical.');
%else
%    Z = (dot(normal, point) - normal(1)*X - normal(2)*Y) / normal(3);
%end

% Plot the plane
figure;
%h = surf(X, Y, Z);
% Basic visualization properties
%set(h, 'EdgeColor', [0.4 0.4 0.4], 'FaceColor', [0 0.7 0.9], 'FaceAlpha', 0.7);

% Plot the normal vector
hold on;
quiver3(0, 0, 0, normal(1), normal(2), normal(3), 'r', 'LineWidth', 2);
quiver3(0, 0, 0, i(1), i(2), i(3), 'b', 'LineWidth', 2);
quiver3(0, 0, 0, j(1), j(2), j(3), 'g', 'LineWidth', 2);
scatter3(0,0,0, 'r+')

% Add labels and title
xlabel('X')
ylabel('Y')
zlabel('Z')

title('Plane π axes');
grid on;

%% 
% Take camera reference axis
X = R(:,1);
Y = R(:,2);
Z = R(:,3);

figure()
hold on
grid on
% Plot camera reference axes in world frame
quiver3(0,0,0, X(1),X(2),X(3), 0.5,'r')
quiver3(0,0,0, Y(1),Y(2),Y(3), 0.5,'g')
quiver3(0,0,0, Z(1),Z(2),Z(3), 0.5,'b')

% Plot world reference axes = matlab frame
quiver3(0,0,0, 1,0,0, 0.5,'y')
quiver3(0,0,0, 0,1,0, 0.5,'m')
quiver3(0,0,0, 0,0,1, 0.5,'c')

legend('X_cam', 'Y_cam', 'Z_cam', 'X_world', 'Y_world', 'Z_world')

xlabel('X')
ylabel('Y')
zlabel('Z')
lim = 0.6;
xlim([-lim,lim])
ylim([-lim,lim])
zlim([-lim,lim])

view(45,45)
%ax = gca;
%ax.XDir = "reverse";
%ax.ZDir = "reverse";

%% PLOT results rotating matlab frame :|
figure
hold on
grid on
% p and q of third frame
p = cam_pts(:,1);
q = cam_pts(:,2);

% plot p
scatter3(p(1), p(2), p(3), 'o', ...
        MarkerEdgeColor="#EDB120", ...
        MarkerFaceColor="#EDB120")
% plot q
scatter3(q(1), q(2), q(3), 'o' , ...
        MarkerEdgeColor="#0072BD", ...
        MarkerFaceColor="#0072BD")
legend('p','q')

pose = rigid3d([1,0,0; 0,1,0; 0,0,1]',[0,0,0]);
plotCamera("AbsolutePose",pose,"Size",2)
% Plot segment between the two feature points
plot3([p(1), q(1)], [p(2), q(2)], [p(3), q(3)], 'k-', ...
      'LineWidth', 2, ...
      'MarkerEdgeColor','none');

xlabel('X');
ylabel('Y');
zlabel('Z');

xlim([-20,20])
ylim([-20,20])
zlim([-20,20])
% Reverse Y and Z axes to match the camera reference system
ax = gca;
%axis equal
ax.YDir = "reverse";
ax.ZDir = "reverse";
% Show XZ plane as the horizontal plane
view([0,1,0]);
%view(0, 90);
title('Camera coordiante frame');

%% PLOT results by inverting Y and Z :)
figure
hold on
grid on
% p and q of third frame
for f=1:3
    p = cam_pts{f}(:,1);
    p = [p(1);
         p(3);
         p(2)];
    q = cam_pts{f}(:,2);
    q = [q(1);
         q(3);
         q(2)];

    plotResultsP2(p,q,R);
end

% plot camera
permutation = [1 0 0;   % rotate R 90 degrees to account for Y-Z switch
               0 0 -1;
               0 1 0];
pose = rigid3d(permutation*R,[0,0,0]);
plotCamera("AbsolutePose",pose,"Size",0.5)

xlabel('X');
ylabel('Z');    % since we have swaped Y with Z ...
zlabel('Y');    % ... and Z with Y

axis equal
%xlim([-20,20])
%ylim([-20,20])
%zlim([-20,20])

ax = gca;
ax.ZDir = "reverse";    % reversing matlab Z-axis,
                        % which for us is the Y-axis (vertical)
view(45, 45);   % modify view point
title('Camera coordiante frame');
