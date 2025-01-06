close all

start = [5 5];
goal = [-5 -5];

heuristic = @(start,goal) norm(start-goal);
x_range = [-10 10];
y_range = [-10 10];

obs_list = [1,-5,2,16];
obs_list = [obs_list; -3 -10 2 15];
obs_list = [obs_list; -6,-4,2,12];
max_iter = 5200;
delta_q = 3;
r_max = 2;
eps = 0.2;
sample_rate = .7;

tic
rrt_star(start,goal,obs_list, heuristic, max_iter, delta_q, r_max, x_range, y_range, eps, sample_rate);
toc

