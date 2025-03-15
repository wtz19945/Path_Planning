close all;
clear;
clc;

map = zeros(50,75);
path = [];

start = [5,1];
goal = [16,74];

% This is the map for the environment excluding the ground obstacles
map(5:end,8:10) = 1;

map(1:22,15:20) = 1;
map(30:35,15:20) = 1;
map(40:end,15:20) = 1;

map(1:15,30:35) = 1;
map(20:25,30:35) = 1;
map(30:35,30:35) = 1;
map(40:end,30:35) = 1;

filename = 'grid.txt';
writematrix(map, filename, 'Delimiter', ' ');

% map(1:15,50:55) = 1;
% map(20:25,50:55) = 1;
% map(30:35,50:55) = 1;
% map(40:end,50:55) = 1;

% This include the information for the ground obstacles
obs_pos  = {[17,43]};
obs_pos{end + 1} = [17,45];
obs_pos{end + 1} = [17,63];
obs_size = {[2,2]};
obs_size{end + 1} = [2,2];
obs_size{end + 1} = [2,5];

obs_info = zeros(size(map));
obs_info(16:19,30:35) = 3;
obs_info(26:29,30:35) = 3;
obs_map = obs_info;

overlap = zeros(size(obs_pos));

for i = 1:length(obs_pos)-1
    for j = i+1:length(obs_pos)
        if check_overlap(obs_pos{i},obs_size{i},obs_pos{j},obs_size{j})
            if overlap(i) == 0
                overlap(i) = 1;
            end
            if overlap(j) == 0
                overlap(j) = 1;
            end
        end
    end
end

for i = 1:length(obs_pos)
    obs_pos_i = obs_pos{i};
    obs_size_i = obs_size{i};

    % Add obstacle information
    for j = -obs_size_i(1):obs_size_i(1)
        for k = -obs_size_i(2):obs_size_i(2)
            obs_map(obs_pos_i(1) + j,obs_pos_i(2) + k) = 5;
        end
    end

    % A condition to test if the obstacle is steppable.
    if overlap(i) == 1
        for j = -obs_size_i(1):obs_size_i(1)
            for k = -obs_size_i(2):obs_size_i(2)
                obs_info(obs_pos_i(1) + j,obs_pos_i(2) + k) = 15; % For overlap
            end
        end
    elseif obs_size_i(2) > 2 
        for j = -obs_size_i(1):obs_size_i(1)
            for k = -obs_size_i(2):obs_size_i(2)
                obs_info(obs_pos_i(1) + j,obs_pos_i(2) + k) = 3; % For large obstacle
            end
        end
    else
        ; % do nothing if the obstacle is small and does not overlap with other obstacles
    end
end

filename = 'foot_obs.txt';
writematrix(obs_info, filename, 'Delimiter', ' ');

weight = [2.0 0;1 2];
% define heuristic
heuristic = @(start,goal) norm(start-goal,2);

% define neighbour
i = -1:1;
j = -1:1;
[gridI, gridJ] = ndgrid(i, j);
neighbor = [gridI(:), gridJ(:)];
% % 4 way connected path
% neighbor = [-1 0;1 0;0 1;0 -1];

%
option = 2;
% A start
tic
path = flip(a_star(start,goal,map, heuristic, neighbor, weight, obs_info, option));
toc

%
visualize_map(map, path, obs_map)