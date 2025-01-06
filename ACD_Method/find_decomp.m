function [a] = find_decomp(polygon_set, tor, result)
%FIND_DECOMP Summary of this function goes here
%   Detailed explanation goes here

set_explored = {};

% Initialize polygon to be explored
% for i = length(polygon_set)
%     set_explored{end + 1} = polygon_set{i};
% end

numColors = length(polygon_set);
colors = lines(numColors);

figure
hold on
for i = 1:length(polygon_set)
    if i == 1
        fill(polygon_set{i}(:, 1), polygon_set{i}(:, 2), colors(i, :), 'FaceAlpha', 0.5, 'EdgeColor', 'blue');
    else
        fill(polygon_set{i}(:, 1), polygon_set{i}(:, 2), [1,1,1], 'FaceAlpha', 1, 'EdgeColor', 'blue');
    end
end
title('input polygon')

for i = 2: length(polygon_set)
    % Compute the principal axis of this polygon
    polygon_cur = polygon_set{i};
    centroid = mean(polygon_cur, 1);
    centeredPolygon = polygon_cur - centroid;
    covarianceMatrix = cov(centeredPolygon);
    [eigVectors, eigValues] = eig(covarianceMatrix);
    
    eig_value = diag(eigValues);
    [~, idx] = max(eig_value);
    PA = eigVectors(:, idx);
    
    % Compute x and cw(x)
    inner_product = centeredPolygon * PA;
    [~,x_idx] = max(inner_product);
    [~,cwx_idx] = min(inner_product);

    x = polygon_cur(x_idx,:);
    cwx = polygon_cur(cwx_idx,:);
    
    % Compute size/ score
    score = norm(x - cwx, 2);
    if score < tor
        continue;
    else
        % merge polygon to 
        dist = inf;
        idx = 0;
        for n = 1 : size(polygon_set{1}, 1)
            cur_x = polygon_set{1}(n,:);
            intersect = false;
            for k = 1: x_idx - 2
                if doSegmentsIntersect([x;cur_x], [polygon_cur(k,:); polygon_cur(k+1,:)])
                    intersect = true;
                    break;
                end
            end
            
            if ~intersect
                for k = x_idx + 1 : size(polygon_cur,1)
                    if k < size(polygon_cur,1)
                        end_idx = k + 1;
                    else
                        end_idx = 1;
                    end
                    if doSegmentsIntersect([x;cur_x], [polygon_cur(k,:); polygon_cur(end_idx,:)])
                        intersect = true;
                        break;
                    end
                end
            end

            new_dist = norm(cur_x- x, 2);

            if new_dist < dist && ~intersect
                idx = n;
                dist = new_dist;
            end
        end

        polygon_set{1}(idx,:)
        % Add children polygon to the main polygon
        norm_vec = polygon_set{1}(idx,:) - polygon_cur(x_idx,:);
        norm_vec = -norm_vec/ norm(norm_vec);
        polygon_set{1} = [polygon_set{1}(1:idx, :); polygon_cur(x_idx:-1:1,:); polygon_cur(end:-1:x_idx + 1,:); ...
            polygon_cur(x_idx,:) - 0.05 * norm_vec; polygon_set{1}(idx,:) + [0.0 0.0];polygon_set{1}(idx+1:end,:)];
    end
    % 
end
% For my case, it is fine to decompose first

% set_explored{end + 1} = polygon_set{1};
set_explored{end + 1} = polygon_set{1};

figure
fill(polygon_set{1}(:, 1), polygon_set{1}(:, 2), 'b' , 'FaceAlpha', 0.5, 'EdgeColor', 'blue');

while ~isempty(set_explored)
    next = set_explored{1};

    hullIndices = convhull(next(:, 1), next(:, 2));
    
    [pocket,max_idx, concave_score] = rank_notch(next);

    if (size(hullIndices,1) - 1 == size(next,1)) || concave_score(max_idx) < tor
        result.Data{end+1} = next;
    else

        [C1,C2] = ACD_2D(next, tor, pocket,max_idx, concave_score);
        
        C1 = C1(1:end-1,:);
        C2 = C2(1:end-1,:);
        if ~isempty(C1)
            if ~is_convex(C1) % Replace this with the concave measurement
                set_explored{end + 1} = C1;
            else
                result.Data{end+1} = C1;
            end
        end

        if ~isempty(C2)
            if ~is_convex(C2) % Replace this with the concave measurement
                set_explored{end + 1} = C2;
            else
                result.Data{end+1} = C2;
            end
        end
    end
    set_explored(1) = [];
end
end

function valid = is_convex(poly)
    index_poly = 1:size(poly,1);
    hullIndices = convhull(poly(:, 1), poly(:, 2));

    valid = size(hullIndices,1) - 1 == size(poly,1);
end

