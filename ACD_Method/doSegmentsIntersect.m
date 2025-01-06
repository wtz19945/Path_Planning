function intersect = doSegmentsIntersect(seg1, seg2)
    % Check if two segments intersect
    % seg1, seg2: 2x2 matrices, each row is a point [x, y]
    % e.g., seg1 = [x1, y1; x2, y2], seg2 = [x3, y3; x4, y4]
    % intersect: true if segments intersect, false otherwise

    % Extract points
    A = seg1(1, :); B = seg1(2, :);
    C = seg2(1, :); D = seg2(2, :);

    % Calculate orientations
    o1 = orientation(A, B, C);
    o2 = orientation(A, B, D);
    o3 = orientation(C, D, A);
    o4 = orientation(C, D, B);

    % General case: Segments intersect if orientations differ
    if o1 ~= o2 && o3 ~= o4
        intersect = true;
        return;
    end

    % Special cases: Check for collinear points
    intersect = (o1 == 0 && isOnSegment(A, C, B)) || ...
                (o2 == 0 && isOnSegment(A, D, B)) || ...
                (o3 == 0 && isOnSegment(C, A, D)) || ...
                (o4 == 0 && isOnSegment(C, B, D));
end

function o = orientation(A, B, C)
    % Calculate orientation of triplet (A, B, C)
    % Returns:
    %  0 -> Collinear
    %  1 -> Clockwise
    %  2 -> Counterclockwise
    val = (B(2) - A(2)) * (C(1) - B(1)) - (B(1) - A(1)) * (C(2) - B(2));
    if abs(val) < 1e-10
        o = 0; % Collinear
    elseif val > 0
        o = 1; % Clockwise
    else
        o = 2; % Counterclockwise
    end
end

function onSegment = isOnSegment(A, C, B)
    % Check if point C lies on segment AB (assuming collinear)
    onSegment = (C(1) >= min(A(1), B(1)) && C(1) <= max(A(1), B(1)) && ...
                 C(2) >= min(A(2), B(2)) && C(2) <= max(A(2), B(2)));
end