function R = rotationMatrixFromVectors(a, b)
% ROTATIONMATRIXFROMVECTORS finds the rotation matrix that rotates vector a to b

    a = a / norm(a);
    b = b / norm(b);
    % Compute axis of rotation v
    v = cross(a, b);
    c = dot(a, b);
    
    if abs(c - 1) < 1e-10
        % a and b are already aligned -> no rotation needed
        R = eye(3); % identity matrix
        return;
    elseif abs(c + 1) < 1e-10
        % a and b are opposite -> no clear rotation axis
        % Choose an arbitrary orthogonal vector which will be rotation axis of a
        axis = null(a');
        % pick one
        v = axis(:,1);
        % Rodrigues formula for 180 deg
        H = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
        R = eye(3) + 2*H*H;
        return;
    end
    
    % General case: Rodrigues formula again
    vx = [    0   -v(3)  v(2);
           v(3)     0  -v(1);
          -v(2)  v(1)     0 ];
      
    R = eye(3) + vx + vx^2 * ((1 - c) / norm(v)^2);
end