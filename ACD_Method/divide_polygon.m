function [subPolygon1, subPolygon2, divides] = divide_polygon(polygon, v1, v2, idx1, idx2, pocket)
    % polygon: Nx2 matrix of polygon vertices (closed polygon, last = first)
    % v1, v2: 1x2 vectors representing the vertices to check
    % isInside: True if the segment is completely inside the polygon
    % dividesPolygon: True if the segment divides the polygon into two parts

    if idx1 < idx2
        subPolygon1 = [polygon(idx1:idx2, :);v1];
        subPolygon2 = [polygon(idx2:end, :); polygon(1:idx1, :);v2];
    else
        subPolygon1 = [polygon(idx2:idx1, :);v2];
        subPolygon2 = [polygon(idx1:end, :); polygon(1:idx2, :);v1];
    end
    
    % Check if the subpolygon is valid
    divides = isValidPolygon(subPolygon1, pocket) && isValidPolygon(subPolygon2, pocket);

end

function valid = isValidPolygon(polygon, pocket)
    % Check if a polygon is simple (no self-intersections)
    [row,col] = size(polygon);
    if inside_pocket(polygon, pocket)
        valid = false;
    else
        if row <= 2
            valid = false;
        elseif row == 3
            valid = true;
        else
            valid = true;
            for n = 1 : row - 2
                %edge1 = polygon(n+1,:) - polygon(n,:);
                seg1 = [polygon(n+1,:);polygon(n,:)];
                for m = n + 2 : row - 2
                    %edge2 = polygon(m+1,:) - polygon(m,:);
                    seg2 = [polygon(m+1,:);polygon(m,:)];
                    if doSegmentsIntersect(seg1, seg2)
                        valid = false;
                    end
                end
            end
        end
    end
end

function valid = inside_pocket(poly, pocket)
    for i = 1:length(pocket)
        pocket_i = pocket{i};
        [~, rowsContained] = ismember(poly, pocket_i, 'rows');
        valid = all(rowsContained);
    end
end

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