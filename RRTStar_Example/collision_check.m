function doesNotIntersect = collision_check(P1, P2, obs_info)
    % P1, P2 are [x, y] points
    % rect is [x_min, x_max, y_min, y_max]
    x_min = obs_info(1); x_max = obs_info(1) + obs_info(3);
    y_min = obs_info(2); y_max = obs_info(2) + obs_info(4);

    % Check if both points are on the same side
    if (P1(1) < x_min && P2(1) < x_min) || ...
       (P1(1) > x_max && P2(1) > x_max) || ...
       (P1(2) < y_min && P2(2) < y_min) || ...
       (P1(2) > y_max && P2(2) > y_max)
        doesNotIntersect = true;
        return;
    end

    % Rectangle edges
    edges = [
        x_min, y_min, x_max, y_min;  % Bottom edge
        x_min, y_max, x_max, y_max;  % Top edge
        x_min, y_min, x_min, y_max;  % Left edge
        x_max, y_min, x_max, y_max;  % Right edge
    ];

    % Check for intersection with each edge
    for i = 1:size(edges, 1)
        if segmentsIntersect(P1, P2, edges(i, 1:2), edges(i, 3:4))
            doesNotIntersect = false;
            return;
        end
    end

    % If no intersection
    doesNotIntersect = true;
end

function intersect = segmentsIntersect(P1, P2, Q1, Q2)
    % Helper function to check if two segments intersect
    intersect = false;

    % Parametric equations
    A = [P2(1) - P1(1), Q1(1) - Q2(1);
         P2(2) - P1(2), Q1(2) - Q2(2)];
    b = [Q1(1) - P1(1); Q1(2) - P1(2)];

    if rank(A) < 2
        return; % Lines are parallel or collinear
    end

    t = A \ b;

    if all(t >= 0 & t <= 1) % Intersection exists within the segments
        intersect = true;
    end
end
