function [] = visualize_map(map, path)
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
    end
end

for i = 1:size(path, 2)
    if i == 1
        plot(path{i}(2) - 0.5, rows - path{i}(1) + 0.5, 'o','MarkerSize',10,    'MarkerEdgeColor','b',...
        'MarkerFaceColor','g');
    else
        plot(path{i}(2) - 0.5, rows - path{i}(1) + 0.5, 'o','MarkerSize',10,    'MarkerEdgeColor','b',...
        'MarkerFaceColor','b');
    end

    if i+1 <= size(path, 2)
        plot([path{i}(2) - 0.5 path{i+1}(2) - 0.5], [rows - path{i}(1) + 0.5 rows - path{i+1}(1) + 0.5], 'LineWidth', 2, 'Color', 'k');
    end
end
% Set axis properties
axis equal;
xlim([0 cols]);
ylim([0 rows]);
set(gca, 'XTick', 0:cols, 'YTick', 0:rows, 'XTickLabel', [], 'YTickLabel', [], 'GridColor', 'k', 'GridAlpha', 0.7);
grid on;
title('Map Visualization');
xlabel('X');
ylabel('Y');
hold off;

end

