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

% map(1:15,50:55) = 1;
% map(20:25,50:55) = 1;
% map(30:35,50:55) = 1;
% map(40:end,50:55) = 1;

% This include the information for the ground obstacles
obs_info = zeros(size(map));
obs_info(16:19,30:35) = 5;
obs_info(26:29,30:35) = 5;

obs_1 = [17,43];
obs_2 = [17,53];
obs_3 = [17,63];

for i = -2:2
    for j = -2:2
        obs_info(obs_1(1) + i,obs_1(2) + j) = 5;
        obs_info(obs_2(1) + i,obs_2(2) + j) = 5;
        obs_info(obs_3(1) + i,obs_3(2) + j) = 5;
    end
end

weight = [2.0 0;1 2];
% define heuristic
heuristic = @(start,goal) norm(start-goal,1);

% define neighbour
i = -1:1;
j = -1:1;
[gridI, gridJ] = ndgrid(i, j);
neighbor = [gridI(:), gridJ(:)];
% % 4 way connected path
neighbor = [-1 0;1 0;0 1;0 -1];

%
option = 2;
% A start
tic
path = flip(a_star(start,goal,map, heuristic, neighbor, weight, obs_info, option));
toc

%
visualize_map(map, path, obs_info)