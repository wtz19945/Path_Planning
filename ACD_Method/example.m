
%% 
close all
% Define a 2D polygon as a list of vertices (clockwise order)
% Example: A non-convex polygon
polygon = {[0, 0; 4, 0; 4, 4; 3, 1;1.5,1.5; 2, 4; 1, 2; 0, 4; 0.5 2]};
polygon = {[0, 0; 4, 0; 4, 4; 0,4]};
% polygon = [0, 0; 1,1; 2,0; 1.5, 1; 2.5,1.25; 1.25, 1.5; 1, 3;.75 1.5;-.5 1.25;0.5 1];

%polygon = [0,0; 1,0; 1,1;0,1];
% polygon = {[0,0; 1,0; 1,1; 0.75, .75; 0.95,.15; .25,.25; .15,.55;1,1;0,1]};
% polygon{end + 1} = [1.5 0.5;3, .25;2.5,0.75;1.0, 1];
% polygon{end + 1} = [0.5,0.5;0.75,0.5;0.75,0.75;0.5,0.75];
polygon{end + 1} = [3.5,0.5;3.75,0.5;3.75,1.5;3.5,1.5];
polygon{end + 1} = [1.5,0.5;1.75,0.5;1.75,1.5;1.5,1.5];
polygon{end + 1} = [3.5,2.5;3.75,2.5;3.75,3.5;3.5,3.5];
polygon{end + 1} = [1.5,2.5;1.75,2.5;1.75,3.5;1.5,3.5];
% Parameters
concavityThreshold = .1; % Concavity threshold for splitting

result = Poly_Data;
% Compute the convex hull
find_decomp(polygon, concavityThreshold, result);

numColors = length(result.Data);
colors = lines(numColors);

figure
hold on
for i = 1:length(result.Data)
    fill(result.Data{i}(:, 1), result.Data{i}(:, 2), colors(i, :), 'FaceAlpha', 0.5, 'EdgeColor', 'blue');
    %
end

title('decomposed polygon')
%%
% Perform decomposition
% [convexParts, splits] = decomposePolygon(polygon, concavityThreshold);

% Plot original polygon and results
% figure;
% subplot(1, 2, 1);
% plotPolygon(polygon, 'Original Polygon');
% % subplot(1, 2, 2);
% % plotDecomposition(convexParts, splits, 'Convex Decomposition');
% 
% function plotPolygon(polygon, titleStr)
%     % Plot a polygon
%     fill(polygon(:, 1), polygon(:, 2), 'cyan', 'FaceAlpha', 0.5, 'EdgeColor', 'blue');
%     hold on;
%     plot(polygon(:, 1), polygon(:, 2), 'bo-');
%     title(titleStr);
%     axis equal;
%     grid on;
% end