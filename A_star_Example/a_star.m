function [path] = a_star(start,goal,map, heuristic, neighbor, weight, obs_info, option)
%A_STAR Summary of this function goes here
%   Detailed explanation goes here
%   start = [x y]
%   goal = [x y]
%   map, 2D grid with 0 as free space, 1 as obstacle
%   heuristic, estimate cost to go
    mapSize = size(map);
    openset = false(mapSize);   % Node remain to be explored next
    closeset = false(mapSize);  % Node already explored 
    f = inf(mapSize);           % Estimate cost of nodes in openset  
    g = inf(mapSize);           % Cost from start to current node
    p = inf(mapSize);           % Parent index
    
    % Add start node to the openset
    openset(start(1),start(2)) = true;
    f(start(1),start(2)) = heuristic(start,goal);
    g(start(1),start(2)) = 0;
    
    path_found = false;
    while any(openset(:) > 0)
        
        % Find the node in openset with smallest cost-to-go
        [~,X] = min(f(:));
        [curX,curY] = ind2sub(mapSize,X);

        % Check if goal is reached
        if norm([curX curY] - goal) == 0
            path_found = true;
            break;
        end

        % Remove current node
        openset(curX,curY) = false;
        f(curX,curY) = inf;
        closeset(curX,curY) = true;
        
        % Explore Neighbors
        next_xy = neighbor + [curX curY];
        for i = 1:size(next_xy, 1)
            if next_xy(i,1) <= 0 || next_xy(i,1) > mapSize(1) || next_xy(i,2) <= 0 || next_xy(i,2) > mapSize(2)
                % If index out of bound
                continue;
            elseif closeset(next_xy(i,1), next_xy(i,2))
                % If index in the closed set
                continue;
            elseif map(next_xy(i,1), next_xy(i,2)) == 1 || (option == 1 && obs_info(next_xy(i,1), next_xy(i,2)) > 0)
                % If the neighbor is a obstacle
                continue;
            else
                temp_g = g(curX,curY) + 1 + obs_info(next_xy(i,1), next_xy(i,2));
                if ~openset(next_xy(i,1), next_xy(i,2))
                    % If it is not in the openlist
                    openset(next_xy(i,1), next_xy(i,2)) = true;
                    p(next_xy(i,1), next_xy(i,2)) = sub2ind(mapSize, curX, curY);
                    g(next_xy(i,1), next_xy(i,2)) = temp_g;
                    f(next_xy(i,1), next_xy(i,2)) = temp_g + weight(1,1) * heuristic([next_xy(i,1), next_xy(i,2)],goal) + ...
                        weight(1,2) * obs_info(next_xy(i,1), next_xy(i,2));
                else
                    if temp_g < g(next_xy(i,1), next_xy(i,2))
                        % If current node is a better path
                        p(next_xy(i,1), next_xy(i,2)) = sub2ind(mapSize, curX, curY);
                        g(next_xy(i,1), next_xy(i,2)) = temp_g;
                        f(next_xy(i,1), next_xy(i,2)) = temp_g + weight(2,1) * heuristic([next_xy(i,1), next_xy(i,2)],goal) + ...
                            weight(2,2) * obs_info(next_xy(i,1), next_xy(i,2));
                    end
                end
            end
        end
    end

    if path_found
        path = {[goal(1), goal(2)]};
    
        % Reconstruct path
        p_ind_cur = p(goal(1), goal(2));
        [px,py] = ind2sub(mapSize,p_ind_cur);
        p_sub_cur = [px,py];
        while norm(p_sub_cur - start) ~= 0
            path{end+1} = p_sub_cur;
            p_ind_cur = p(px, py);
            [px,py] = ind2sub(mapSize,p_ind_cur);
            p_sub_cur = [px,py];
        end
        path{end+1} = start;
    else
        path = {};
    end
    
end

