function H = DLT(m1,m2)
    % DLT - Implement Direct Linear Transform to determine the homography
    % that translates points in m1 to their equivalent in m2
    %
    % Inputs:
    %   m1 - [3xN] matrix containing destination points
    %   m2 - [3xN] matrix containing source points
    %   
    % Outputs:
    %   H - homography matrix describing the transformation from m1 to m2
    %
    % Build the  matrix of the linear homogeneous system
    % A vec(H) = 0
    num_points = 4;
    A = zeros(2*num_points,9);
    % the rows are the number of independent constraints
    % the columns are the number of unknown (9 for a 3x3 matrix)
    for i =1:num_points
        p1=m1(:,i);
        p2=m2(:,i);
        A(2*i-1,:) = [zeros(1,3), p1'*p2(3), - p1'*p2(2)];
        A(2*i,:) = [p2(3)*p1', zeros(1,3), - p1'*p2(1)];
    end
    % now we have to solve an homogeneous system꞉ svd!
    [u,s,v] = svd(A);
    % let's have a look at how well the system is conditioned
    s = diag(s);
    % rough estimate of the nullspace dimension
    nullspace_dimension = sum(s < eps * s(1) * 1e3);
    if (nullspace_dimension > 1)
        fprintf('Nullspace is too large to accomodate a single solution...\n');
    end
    h = v(:,end);
    H = reshape(h,3,3)';
    H = H./norm(H);
end

