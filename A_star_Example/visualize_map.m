function [] = visualize_map(map, path, obs_info)
%VISUALIZE_MAP Summary of this function goes here
%   Detailed explanation goes here

% Get the map size
[rows, cols] = size(map);

% Plot the map
figure;

hold on;
for i = 1:rows
    for j = 1:cols
        if map(i, j) == 1
            % Plot red square for obstacles
            rectangle('Position', [j-1, rows-i, 1, 1], 'FaceColor', 'r', 'EdgeColor', 'k');
        else
            % Plot white square for free space
            rectangle('Position', [j-1, rows-i, 1, 1], 'FaceColor', 'w', 'EdgeColor', 'k');
        end

        if obs_info(i, j) > 0
            rectangle('Position', [j-1, rows-i, 1, 1], 'FaceColor', 'y', 'EdgeColor', 'k');
        end
    end
end

patch(NaN, NaN, 'r', 'DisplayName', 'Wall');
patch(NaN, NaN, 'w', 'DisplayName', 'Free Space');
patch(NaN, NaN, 'y', 'DisplayName', 'Ground Object');

path = cell2mat(path);
path = reshape(path,2, length(path)/2);
plot(path(2,:) - 0.5, rows - path(1,:) + 0.5, 'o','MarkerSize',10,    'MarkerEdgeColor','b',...
'MarkerFaceColor','b', 'DisplayName', 'Path');
% Set axis properties
axis equal;
xlim([0 cols]);
ylim([0 rows]);
set(gca, 'XTick', 0:cols, 'YTick', 0:rows, 'XTickLabel', [], 'YTickLabel', [], 'GridColor', 'k', 'GridAlpha', 0.7);
grid on;
title('Map Visualization');
xlabel('X');
ylabel('Y');
legend('show')
hold off;

end

