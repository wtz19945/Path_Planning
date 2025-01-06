% Create a figure
figure;

% Plot a dummy patch or scatter for the color representation
hold on;
obstacleColor = [1, 0, 0]; % Red color for obstacle
fill([-1, 1, 1, -1], [-1, -1, 1, 1], obstacleColor, 'EdgeColor', 'none', 'DisplayName', 'Obstacle');

% Plot other elements in the figure
plot(0, 0, 'bo', 'DisplayName', 'Start Point'); % Example point
plot(5, 5, 'gx', 'DisplayName', 'Goal Point'); % Example point

% Add a legend
legend('show');
hold off;