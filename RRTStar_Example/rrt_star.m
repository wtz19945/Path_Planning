function [path] = rrt_star(start,goal,obs_list, heuristic, max_iter, delta_q, r_max, x_range, y_range, eps, sample_rate)
%A_STAR Summary of this function goes here
%   Detailed explanation goes here
%   start = [x y]
%   goal = [x y]
%   obs, a list of obstacles
%   heuristic, estimate cost to go
%   max_iter : maximum iteration allowed
%   delta_q  : maximum step size
%   r_max    : search radius for rewire

V = start;
E = [];
cost = 0;

figure;
hold on;
grid on;
axis([x_range(1) x_range(2) y_range(1) y_range(2)]);
plot(start(1), start(2), 'ro', 'MarkerSize', 10, 'DisplayName', 'Start');
plot(goal(1), goal(2), 'go', 'MarkerSize', 10, 'DisplayName', 'Goal');
%legend;

for i = 1:size(obs_list,1)
    obs = obs_list(i, :);
    rectangle('Position',obs(1,:),'FaceColor',[0 .5 .5])
end

dim = 2; % Dimensionality
samples = haltonset(dim);
samples = net(samples, max_iter); % Generate Halton samples

for iter = 1:max_iter
    % Sample a point
    rn = rand;
    if rn < 1 - sample_rate
        % Sample around the goal
        x_rand = goal + delta_q * (rand(1,2) - 0.5);
    elseif rn > sample_rate
        dist_point = vecnorm(V - goal, 2, 2);
        [~, idx_nearest] = min(dist_point);
        goal2 = V(idx_nearest, :);
        x_rand = goal2 + delta_q * (rand(1,2) - 0.5);
    else
        %x_rand = [(rand - 0.5) * (x_range(2) - x_range(1)) (rand - 0.5) * (y_range(2) - y_range(1))];
        x_rand = [(samples(iter,1) - 0.5) * (x_range(2) - x_range(1)) (samples(iter,2)  - 0.5) * (y_range(2) - y_range(1))];
    end

    % Find the nearest point in the tree
    dist_point = vecnorm(V - x_rand, 2, 2);
    [~, idx_nearest] = min(dist_point);

    % Compute the search direction
    x_near = V(idx_nearest,:);
    direction = (x_rand - x_near) / norm(x_rand - x_near);

    % New candidate point
    x_new = x_near + delta_q * direction;

    % TODO: Check if x_near to x_new is collision-free
    collision = false;
    for i = 1:size(obs_list,1)
        obs = obs_list(i, :);
        if (x_new(1) >= obs(1) && x_new(1) <= obs(1) + obs(3) && x_new(2) >= obs(2) && x_new(2) <= obs(2) + obs(4)) ...
                || ~collision_check(x_new, x_near, obs)
            collision = true;
            break;
        end
    end

    if collision
        continue;
    end
    
    % Find nodes near the x_new within radius r_max
    distances = vecnorm(V - x_new, 2, 2);
    nearby_idx = find(distances < r_max);

    % Check if x_new can be reached via another point in the tree with
    % lower cost
    min_cost = cost(idx_nearest) + heuristic(x_new, x_near) + norm(x_new - goal);
    best_parent_idx = idx_nearest;

    for in = i:length(nearby_idx)
        idx = nearby_idx(in);
        % Only check collision-free path
        collision = false;
        for i = 1:size(obs_list,1)
            obs = obs_list(i, :);
            if ~collision_check(x_new, V(idx,:), obs)
                collision = true;
                break;
            end
        end
    
        if collision
            continue;
        end
        new_cost = cost(idx) + heuristic(x_new, V(idx,:))  + norm(x_new - goal);
        if new_cost < min_cost
            min_cost = new_cost;
            best_parent_idx = idx;
        end
    end
    
    delta_q = max(delta_q * 0.99,0.4);
    % Add x_new to the tree
    V = [V;x_new];
    E = [E;best_parent_idx size(V,1)];
    cost = [cost, min_cost];

    % Rewire
    if mod(iter,5) == 0
        for in = i:length(nearby_idx)
            idx = nearby_idx(in);
            % Only check collision-free path
            collision = false;
            for i = 1:size(obs_list,1)
                obs = obs_list(i, :);
                if ~collision_check(x_new, V(idx,:), obs)
                    collision = true;
                    break;
                end
            end
            if collision
                continue;
            end

            if idx == best_parent_idx
                continue;
            end
            new_cost = cost(end) + heuristic(x_new, V(idx,:)) + norm(V(idx,:) - goal);
            if new_cost < cost(idx)
                cost(idx) = new_cost;
                % Update edge
                E(E(:, 2) == idx, :) = [size(V,1) , idx];
            end
        end
    end

    if norm(x_new - goal) < eps
        % V = [V; goal];
        % E = [E; size(V, 1) - 1, size(V, 1)];
        % cost = [cost, cost(end) + norm(x_new - goal)];
        % disp('Goal reached!');
        break;
    end
    % Plot the tree and new edges
    plot([x_near(1), x_new(1)], [x_near(2), x_new(2)], 'b-');
    %drawnow;
end

path = [size(V, 1)];
while path(1) ~= 1
    path = [E(path(1) - 1,1), path];
    plot([V(path(1), 1), V(path(2), 1)], [V(path(1), 2), V(path(2), 2)], 'r-', 'LineWidth', 2);
end


for i = 2:length(path)
    plot([V(path(i-1), 1), V(path(i), 1)], [V(path(i-1), 2), V(path(i), 2)], 'r-', 'LineWidth', 2);
end

end

