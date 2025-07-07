function [P_list, Q_list, normal] = localize_car(K, image_points, d)
% LOCALIZE_CAR_POINTS Estimate 3D points of car taillights from 3 frames
%
% Inputs:
%   - K : 3x3 camera intrinsic matrix
%   - image_points : 3x2x3 array (3 frames, 2 points: p and q, 2D coords + 1 homogeneous coordinate)
%                    Format: image_points(i, j, :) — i=frame, j=1 is p, j=2 is q
%   - d : known real-world distance between tail lights
%
% Outputs:
%   - P_list, Q_list: 3x3 matrices containing 3D points for each frame
%   - normal: estimated unit normal vector of the plane

    % Step 1: Convert image points to normalized back-projection rays
    num_frames = size(image_points, 1);
    d_p = zeros(num_frames, 3);  % Direction vectors for p
    d_q = zeros(num_frames, 3);  % Direction vectors for q
    
    for i = 1:num_frames
        % Extract points
        p = squeeze(image_points(i, 1, :));  % 2x1
        q = squeeze(image_points(i, 2, :));
        
        % Backproject to rays in camera coordinates
        dp_i = K \ p;
        dq_i = K \ q;
        
        % Normalize
        d_p(i, :) = dp_i / norm(dp_i);
        d_q(i, :) = dq_i / norm(dq_i);
    end

    % Step 2: Optimize the plane normal to preserve the segment length d
    % Initial guess: normal pointing forward
    n0 = [0; 1; 1];
    n0 = n0 / norm(n0);
    
    % Optimize using fmincon with unit norm constraint
    options = optimoptions('fmincon', 'Display', 'iter', 'Algorithm', 'sqp');
    problem = createOptimProblem('fmincon', ...
        'x0', n0, ...
        'objective', @(n) segment_length_error(n, d_p, d_q, d), ...
        'nonlcon', @unit_norm_constraint, ...
        'options', options);
    
    % Solve
    [normal, ~] = fmincon(problem);
    normal = normal/norm(normal);

    % Make sure the points stays in front of the camera
    if dot(normal, d_p(1,:)) < 0
       normal = -normal;
    end

    % Step 3: Compute 3D points by intersecting rays with the plane
    P_list = zeros(num_frames, 3);
    Q_list = zeros(num_frames, 3);
    
    for i = 1:num_frames
        
        lambda_p = 1 / (dot(normal, d_p(i, :)));
        lambda_q = 1 / (dot(normal, d_q(i, :)));

        if lambda_p < 0 || lambda_q < 0
            warning('Negative scale — ray/plane intersection error.');
        end
        
        P_list(i, :) = lambda_p * d_p(i, :);
        Q_list(i, :) = lambda_q * d_q(i, :);
    end

    % Correct error on lenghts
    %for i = 1:3
    %    seg = Q_list(i,:) - P_list(i,:);
    %    seg_dir = seg / norm(seg);
    %    mid = (Q_list(i,:) + P_list(i,:)) / 2;
    %    P_list(i,:) = mid - 0.5 * d * seg_dir;
    %    Q_list(i,:) = mid + 0.5 * d * seg_dir;
    %end

end

% Cost function: Sum of squared differences from expected segment length d
function E = segment_length_error(n, d_p, d_q, d)
    expected_normal = [0,1,0];
    alpha = 2;
    E = 0;
    n = n(:);  % Ensure column vector
    for i = 1:size(d_p, 1)
        lambda_p = 1 / (dot(n, d_p(i, :)));
        lambda_q = 1 / (dot(n, d_q(i, :)));
        
        P = lambda_p * d_p(i, :);
        Q = lambda_q * d_q(i, :);
        
        dist = norm(P - Q);
        E = E + (dist - d)^2;
    end
    E = E + alpha*(1 - dot(n, expected_normal))^2;
end

% Constraint: normal vector must be unit length
function [c, ceq] = unit_norm_constraint(n)
    c = []; % No inequality constraints
    ceq = norm(n) - 1;
end
