function R = plotResults_P2(p, q)
    % Car dimensions
    length = 3.538;    % front-to-back (forward)
    width = 1.578;     % left-to-right
    height = 1.578;    % bottom-to-top
    offset_ground = 0.948; % [m] from taillight to ground

    % Vector from p (left taillight) to q (right taillight)
    u = q - p;                  % right (local X)
    u = u / norm(u);

    % Estimate forward direction as orthogonal to u and up
    up_world = [0; 1; 0];       % Y is up in our swapped frame
    w = cross(up_world, u);     % forward (local Z)
    w = w / norm(w);

    v = cross(w, u);            % corrected up (local Y)
    v = v / norm(v);

    % Car local frame: [right (u), up (v), forward (w)]
    R = [u, v, w];

    % Rear-left-bottom corner is p lowered to ground level
    origin = p;

    % Car box defined in local frame
    car_local = [ 0         0       -offset_ground;
                  norm(q-p) 0       -offset_ground;
                  0         -length -offset_ground;
                  norm(q-p) -length -offset_ground;
                  0         0       height-offset_ground;
                  norm(q-p) 0       height-offset_ground;
                  0         -length height-offset_ground;
                  norm(q-p) -length height-offset_ground ]';

    % Transform to global frame
    car_global = R * car_local + origin;

    % Extract coordinates
    X = car_global(1,:);
    Y = car_global(2,:);
    Z = car_global(3,:);

    % Box faces (6 patches)
    faces = [
        1 2 4 3;  % back
        5 6 8 7;  % front
        1 3 7 5;  % left
        2 4 8 6;  % right
        3 4 8 7;  % top
        1 2 6 5   % bottom
    ];

    % Draw the car box
    for k = 1:size(faces,1)
        patch('XData', X(faces(k,:)), ...
              'YData', Y(faces(k,:)), ...
              'ZData', Z(faces(k,:)), ...
              'FaceColor', 'c', 'FaceAlpha', 0.5);
    end

    % Draw taillight points and rear axle
    scatter3(p(1),p(2),p(3), 'ro');
    scatter3(q(1),q(2),q(3), 'ro');
    plot3([p(1), q(1)], [p(2), q(2)], [p(3), q(3)], 'k-', 'LineWidth', 2);
end
