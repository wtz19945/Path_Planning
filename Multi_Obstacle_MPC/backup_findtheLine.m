       % x_pos = design_vector(1:2:Nodes * 2);
        % y_pos = design_vector(3*Nodes + Npred + 1:2:5*Nodes + Npred);
        % 
        % % Compute A, B, C the line
        % start_point = [-y_pos(1);x_pos(1)];
        % end_point = [-y_pos(1);x_pos(end)] - start_point;
        % 
        % % Compute list of obstacles
        % obs_list = {};
        % obs_list{end + 1} = [-swf_obs_pos(2) - start_point(1) swf_obs_pos(1) - start_point(2)  swf_xy_r(2) swf_xy_r(2)];
        % 
        % % Compute list of line boundaries
        % start_point = zeros(2,1);
        % point_vec = end_point - start_point;
        % theta = 90; % to rotate 90 counterclockwise
        % R = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
        % norm_vec = R * point_vec;
        % A = norm_vec(1);
        % B = norm_vec(2);
        % C = -norm_vec' * start_point;
        % line_list = {};
        % line_list{end + 1} = [A B C end_point(1) end_point(2)];
        % line_list{end + 1} = [A B -norm_vec' * [0.15;0.0] end_point(1) + 0.15 end_point(2)];
        % line_list{end + 1} = [A B -norm_vec' * [-0.15;0] end_point(1) - 0.15 end_point(2)];
        % 
        % % Plot the figure
        % h2 = figure(999);  % Use a unique handle for the new figure
        % clf(h2);  % Clear the figure to avoid overlap
        % 
        % weight = [1 2.5 0.1]; % Optimization weights on Number of obstacle intersected, angle changed, and intersected length
        % N = 100;              % Number of candidates
        % T = 15;               % Number of parents candidates
        % sigma = .1;           % Pertubation Magnitude
        % params = [N T sigma];
        % plot_map(obs_list, line_list)  
        % elite = run_ga_multi_obstacle(weight, line_list, obs_list, params);
        % elite_vec = [elite_vec;elite];
        % 
        % plot(y_pos- y_pos(1), x_pos- x_pos(1))
        % 
        % % Plot the results
        % rot = [cos(elite) -sin(elite); sin(elite) cos(elite)];
        % new_vec = rot * [A;B];
        % C = -new_vec' * [0;0];
        % A = new_vec(1);
        % B = new_vec(2);
        % 
        % if A ~= 0 && B ~= 0
        %     x_line = linspace(-2,2,100);
        %     y_line = (-A * x_line - C) / B;
        % elseif A == 0 && B ~= 0
        %     x_line = linspace(-2,2,100);
        %     y_line = (-A * x_line - C) / B;
        % elseif A ~= 0 && B == 0
        %     y_line = linspace(-2,2,100);
        %     x_line = (-B * y_line - C) / A;
        % else
        %     error('not a valid line');
        % end
        % plot(x_line, y_line,'LineWidth',5);
        % 
        % drawnow
        % title('Additional Figure, Step ');