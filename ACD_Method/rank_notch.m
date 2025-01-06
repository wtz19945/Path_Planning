function [pocket,max_idx, concave_score] = rank_notch(polygon)
%RANK_NOTCH Summary of this function goes here
%   Detailed explanation goes here
index_poly = 1:size(polygon,1);

hullIndices = convhull(polygon(:, 1), polygon(:, 2));
notch_index = setdiff(index_poly, hullIndices(1:end));
concave_score = zeros(size(polygon,1), 1);

pocket = {};
% Compute Concavity of notch points
for idx = notch_index
    pocket_idx = idx;
    left_idx  = idx;
    right_idx = idx;
    while ~any(hullIndices == left_idx)
        left_idx = left_idx - 1;
        if left_idx <= 0
            left_idx = left_idx + size(polygon,1);
        end
        pocket_idx = [pocket_idx;left_idx];
    end
    while ~any(hullIndices == right_idx)
        right_idx = right_idx + 1;
        if right_idx > size(polygon,1)
            right_idx = right_idx - size(polygon,1);
        end
        pocket_idx = [pocket_idx;right_idx];
    end
    
    pocket{end + 1} = [polygon(pocket_idx,:)];
    % Compute
    notch = polygon(idx,:);
    bound_l = polygon(left_idx,:);
    bound_r = polygon(right_idx,:);
    concave = dis2line(bound_l',bound_r', notch');

    concave_score(idx) = concave;
end

[max_convace, max_idx] = max(concave_score);
end

