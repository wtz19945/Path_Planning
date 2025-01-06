function plotPrincipalAxis(polygon)
    % Compute principal axis of a polygon and plot it
    % polygon: Nx2 matrix representing vertices of the polygon

    % Step 1: Compute centroid of the polygon
    centroid = mean(polygon, 1);

    % Step 2: Center the polygon by subtracting the centroid
    centeredPolygon = polygon - centroid;

    % Step 3: Compute the covariance matrix of the centered polygon
    covarianceMatrix = cov(centeredPolygon);

    % Step 4: Compute the eigenvalues and eigenvectors of the covariance matrix
    [eigVectors, eigValues] = eig(covarianceMatrix);

    % Step 5: Extract principal axes
    principalAxis1 = eigVectors(:, 1); % First principal axis
    principalAxis2 = eigVectors(:, 2); % Second principal axis

    % Step 6: Scale eigenvectors for visualization
    scaleFactor = sqrt(diag(eigValues));
    axis1 = principalAxis1' * scaleFactor(1);
    axis2 = principalAxis2' * scaleFactor(2);
    % Step 7: Plot the polygon
    figure;
    hold on;
    fill(polygon(:, 1), polygon(:, 2), 'cyan', 'FaceAlpha', 0.5, 'EdgeColor', 'blue');
    scatter(centroid(1), centroid(2), 'r', 'filled'); % Centroid

    % Step 8: Plot principal axes
    quiver(centroid(1), centroid(2), axis1(1), axis1(2), 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Principal axis 1
    quiver(centroid(1), centroid(2), axis2(1), axis2(2), 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 0.5); % Principal axis 2

    % Formatting
    title('Principal Axes of Polygon');
    xlabel('X');
    ylabel('Y');
    axis equal;
    grid on;
    hold off;
end

% Example usage
polygon = [1.5 0.5;2.5, .25;2.5,0.75;1.5, 1]; % Square
plotPrincipalAxis(polygon);
