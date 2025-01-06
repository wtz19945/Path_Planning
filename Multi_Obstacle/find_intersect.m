function [intersect,inter_point] = find_intersect(line_info,ellipse_info)
%CHECK_INTERSECT Summary of this function goes here
%   Detailed explanation goes here
% Get information
A = line_info(1); 
B = line_info(2);
C = line_info(3);

h = ellipse_info(1);
k = ellipse_info(2);
r1 = ellipse_info(3);
r2 = ellipse_info(4);

intersect = true;
inter_point = [];

if A ~= 0 && B ~= 0
    a1 = 1 / r1^2 + A^2 / (r2^2 * B^2);
    a2 = -1/r1^2 * 2 * h + 1/r2^2 * 2 * (C/B + k) * A / B;
    a3 = h^2/r1^2 + 1/r2^2 * (C/B + k)^2 - 1;
    
    temp = a2^2 - 4 * a1 * a3;
    if temp > 0
        x1 = (-a2 + sqrt(temp)) / (2 * a1);
        x2 = (-a2 - sqrt(temp)) / (2 * a1);
    
        % Solver for y
        y1 = (-A*x1 - C) / B;
        y2 = (-A*x2 - C) / B;

        inter_point = [x1 y1 x2 y2];
    else
        intersect = false;
        %disp('does not intersect')
    end
elseif A == 0 && B ~= 0
    % y = -C/B
    y1 = -C/B;
    y2 = -C/B;
    if y1 >= k - r2 && y1 <= k + r2
        x1 = -sqrt(r1^2 - r1^2/r2^2 * (y1 - k)^2) + h;
        x2 = sqrt(r1^2 - r1^2/r2^2 * (y1 - k)^2) + h;
        inter_point = [x1 y1 x2 y2];
    else
        intersect = false;
        %disp('does not intersect')
    end
elseif A ~= 0 && B == 0
    % x = -C/A
    x1 = -C/A;
    x2 = -C/A;
    if x1 >= h - r1 && x1 <= h + r1
        y1 = -sqrt(r2^2 - r2^2/r1^2 * (x1 - h)^2) + k;
        y2 = sqrt(r2^2 - r2^2/r1^2 * (x1 - h)^2) + k;
        inter_point = [x1 y1 x2 y2];
    else
        intersect = false;
        %disp('does not intersect')
    end
else
    error('not a valid line');
end

end

