function [A, b, C,d] = Find_obs_free_region(C,d,obs_list, bound, max_iter, tol)
%FIND_OBS_FREE_REGION Summary of this function goes here
%   Detailed explanation goes here

iter = 1;
while iter < max_iter
    figure
    hold on

    for i = 1 : length(obs_list)
        scatter(obs_list{i}(:,1), obs_list{i}(:,2), 50, 'b', 'filled', 'DisplayName', 'Vertices');
        hullIndices = convhull(obs_list{i}(:,1), obs_list{i}(:,2)); % Indices of the convex hull
        convexHullVertices = obs_list{i}(hullIndices, :); % Extract vertices of the convex hull
        plot(convexHullVertices(:,1), convexHullVertices(:,2), 'r-', 'LineWidth', 2, 'DisplayName', 'Convex Hull');
        plot(d(1), d(2), 'o','MarkerSize',10,    'MarkerEdgeColor','b',...
            'MarkerFaceColor','g');
        xlim([-5 5]);
        ylim([-5 5])
    end

    [A,b, p] = seperate_hyplanes(C, d, obs_list);
    [Cnew,d] = InscribedEllipse(A, b, bound);

    % Draw seperate plane
    for i = 1 : length(b)
        plot(p(i, 1), p(i,2), 'o','MarkerSize',10,    'MarkerEdgeColor','b',...
            'MarkerFaceColor','g');

        plot([p(i, 1), p(i, 1) + A(i,1)], [p(i, 2), p(i, 2) + A(i,2)])
    end

    % Draw ellipse
    n = 100;
    theta = linspace(0, 2*pi, n);  % Angle parameter
    x = [cos(theta); sin(theta)];    % Points on the unit circle
    ellipse_points = Cnew * x + d;
    plot(ellipse_points(1, :), ellipse_points(2, :), 'b-', 'LineWidth', 2); % Ellipse
    plot(d(1), d(2), 'ro', 'MarkerSize', 8, 'LineWidth', 2); % Center of ellipse

    % Check stop criteria
    if det(Cnew - C) / det(C) < tol
        disp('converged')
        break;
    end
    C = Cnew;
    iter = iter + 1;
end
end

