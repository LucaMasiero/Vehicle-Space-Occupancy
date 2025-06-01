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
[file,location] = uigetfile({'*.*'; '*jpg'},'MultiSelect', 'on');
if isequal(file,0)
   disp('User selected Cancel');
else
   disp('Features selected');
end

feature_pts = {};
for i=1:3
    abs_path = fullfile(location,file(i));
    feature_pts{i} = selectFeatures_p2(abs_path{1}, cameraParams.Intrinsics);
end

%%
syms a b

n = [a 1 b]'/sqrt(a^2 + b^2 + 1);
i = [1 -a 0]'/sqrt(a^2 + 1);
j = [-a*b -b a^2+1]'/(sqrt(a^2*b^2 + b^2 + a^4 + 1));

p = sym(zeros(2,3));
q = sym(zeros(2,3));
for k=1:3
    p_prime = [feature_pts{k}(1,:), 1]';
    q_prime = [feature_pts{k}(2,:), 1]';

    p(:,k) = [i'; j']*(inv(K)*p_prime)/(n'*inv(K)*p_prime);
    q(:,k) = [i'; j']*(inv(K)*q_prime)/(n'*inv(K)*q_prime);
end

% Define system
eqn1 = norm(p(:,1)-q(:,1)) == norm(p(:,2)-q(:,2));
eqn2 = norm(p(:,3)-q(:,3)) == norm(p(:,2)-q(:,2));
%eqn1 = norm(p(:,1)-q(:,1))==114.5;
%eqn2 = norm(p(:,2)-q(:,2))==114.5;
%eqn3 = norm(p(:,3)-q(:,3))==114.5;
%eqn3 = dot(i,j)==0;
%eqn4 = dot(n,j)==0;
eqns = [eqn1,eqn2];

solutions = solve(eqns, [a b])
%%
a = double(solutions.a);
b = double(solutions.b);
n_sol = [a 1 b]'/sqrt(a^2 + b^2 + 1);
i = [1 -a 0]'/sqrt(a^2 + 1);
j = [a*b -b a^2+1]'/(sqrt(a^2 + b^2 + 1)*sqrt(a^2 + 1));

%%
% Real taillight positions in car coordinate frame
tail_light_distance = 121;
car_pts = [
    -tail_light_distance/2, 0, 0;  % Left taillight
     tail_light_distance/2, 0, 0   % Right taillight
]';

% From your loop
p3D = inv(K) * p_prime / (n_sol' * inv(K) * p_prime);
q3D = inv(K) * q_prime / (n_sol' * inv(K) * q_prime);

cam_pts = [p3D, q3D];  % 3x2 matrix

% Use Kabsch algorithm or Procrustes (no scale)
[U, ~, V] = svd(cam_pts * car_pts');
R = U * V'
t = mean(cam_pts, 2) - R * mean(car_pts, 2)

%% Plot

% Define the normal vector with high precision values
normal = n_sol;

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