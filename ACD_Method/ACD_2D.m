function [C1_cand,C2_cand] = ACD_2D(polygon, tor, pocket,max_idx, concave_score)
%ACD_2D Summary of this function goes here
%   Detailed explanation goes here

index_poly = 1:size(polygon,1);
r = polygon(max_idx,:);

%
score = -inf;
index = 0;
C1_cand = [];
C2_cand = [];
for idx = index_poly
    x = polygon(idx,:);
    % Check if x resolve y
    if norm(idx - max_idx) <= 1
        % skip notch itself or the nearby vertex
        continue;
    end

    % If self collision
    % intersect = false;
    % large_one = max(idx, max_idx);
    % small_one = min(idx, max_idx);
    % 
    % for k = 1: small_one - 2
    %     if doSegmentsIntersect([x;r], [polygon(k,:); polygon(k+1,:)])
    %         intersect = true;
    %         break;
    %     end
    % end
    % 
    % for k = small_one + 1 : large_one - 2
    %     if doSegmentsIntersect([x;r], [polygon(k,:); polygon(k+1,:)])
    %         intersect = true;
    %         break;
    %     end
    % end
    % 
    % if ~intersect
    %     for k = large_one + 1 : size(polygon,1)
    %         if k < size(polygon,1)
    %             end_idx = k + 1;
    %         else
    %             end_idx = 1;
    %         end
    %         if doSegmentsIntersect([x;r], [polygon(k,:); polygon(end_idx,:)])
    %             intersect = true;
    %             break;
    %         end
    %     end
    % end
    % 
    % if intersect
    %     continue;
    % end
    [C1,C2,valid] = divide_polygon(polygon, r, x, max_idx, idx, pocket);
    % C1
    % C2
    % if valid
    % figure
    % fill(polygon(:, 1), polygon(:, 2), 'c', 'FaceAlpha', 0.5, 'EdgeColor', 'blue');
    % axis equal;
    % figure
    % fill(C1(:, 1), C1(:, 2), 'c', 'FaceAlpha', 0.5, 'EdgeColor', 'blue');
    % axis equal;
    % figure
    % fill(C2(:, 1), C2(:, 2), 'k', 'FaceAlpha', 0.5, 'EdgeColor', 'blue');
    % axis equal;
    % end
    if valid
        new_score = (1 + 0.1 * concave_score(idx)) / (norm(x - r, 2));
        if new_score > score
            index = idx;
            score = new_score;
            C1_cand = C1;
            C2_cand = C2;
        end
    else
    end
end
% ACD_2D(C1_cand, tor, result);
% ACD_2D(C2_cand, tor, result);
% 
% figure
% fill(C1_cand(:, 1), C1_cand(:, 2), 'c', 'FaceAlpha', 0.5, 'EdgeColor', 'blue');
% hold on
% fill(C2_cand(:, 1), C2_cand(:, 2), 'k', 'FaceAlpha', 0.5, 'EdgeColor', 'blue');
% axis equal;

end

