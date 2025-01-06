% list of obstacles
close all

% Initial point
q0 = [0;1.5];
d = q0;
eps = 1e-3;
C = eps * [1 0;0 1];


obs_vertex = {[2 2; 1 3; 3 3; 3 4]};
obs_vertex{end + 1} = [-2 1; -3 2; -0 0; -1 3];
obs_vertex{end + 1} = [-2 -2; -3 -1; -1 -1; -1 0];
obs_vertex{end + 1} = [1 -2; 0 -1; 2 -1; 1 1];
obs_vertex{end + 1} = [3 -4; 2 -3; 4 -3; 4 -2];
obs_vertex{end + 1} = [-4 4; -3 3; 2 7; 1 8];
obs_vertex{end + 1} = [5 2; 6 3; 3 7;4 8];

[A,b, C1,d1] = Find_obs_free_region(C, d, obs_vertex, 0.01, 10, 1e-3);

obs_list = obs_vertex;


q02 = [2;5];
d = q02;
eps = 1e-3;
C = eps * [1 0;0 1];


obs_vertex = {[2 2; 1 3; 3 3; 3 4]};
obs_vertex{end + 1} = [-2 1; -3 2; -0 0; -1 3];
obs_vertex{end + 1} = [-2 -2; -3 -1; -1 -1; -1 0];
obs_vertex{end + 1} = [1 -2; 0 -1; 2 -1; 1 1];
obs_vertex{end + 1} = [3 -4; 2 -3; 4 -3; 4 -2];
obs_vertex{end + 1} = [-4 4; -3 3; 2 7; 1 8];
obs_vertex{end + 1} = [5 2; 6 3; 3 7;4 8];

[A2, b2, C2,d2] = Find_obs_free_region(C, d, obs_vertex, 0.01, 10, 1e-3);

figure
hold on

for i = 1 : length(obs_list)
    scatter(obs_list{i}(:,1), obs_list{i}(:,2), 50, 'b', 'filled', 'DisplayName', 'Vertices');
    hullIndices = convhull(obs_list{i}(:,1), obs_list{i}(:,2)); % Indices of the convex hull
    convexHullVertices = obs_list{i}(hullIndices, :); % Extract vertices of the convex hull
    plot(convexHullVertices(:,1), convexHullVertices(:,2), 'r-', 'LineWidth', 2, 'DisplayName', 'Convex Hull');
end

%
plot(q0(1), q0(2), 'o','MarkerSize',10,    'MarkerEdgeColor','b',...
    'MarkerFaceColor','g');
plot(q02(1), q02(2), 'o','MarkerSize',10,    'MarkerEdgeColor','b',...
    'MarkerFaceColor','g');
xlim([-5 5]);
ylim([-5 5])

% Draw ellipse
n = 100;
theta = linspace(0, 2*pi, n);  % Angle parameter
x = [cos(theta); sin(theta)];    % Points on the unit circle
ellipse_points = C1 * x + d1;
plot(ellipse_points(1, :), ellipse_points(2, :), 'b-', 'LineWidth', 2); % Ellipse
plot(d(1), d(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2); % Center of ellipse

% Draw ellipse
n = 100;
theta = linspace(0, 2*pi, n);  % Angle parameter
x = [cos(theta); sin(theta)];    % Points on the unit circle
ellipse_points = C2 * x + d2;
plot(ellipse_points(1, :), ellipse_points(2, :), 'b-', 'LineWidth', 2); % Ellipse
plot(d(1), d(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2); % Center of ellipse
title('final result')
