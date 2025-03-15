clear
close all;
clc

% Define line as Ax + By + C = 0
A = 0; 
B = 1;
C = 0;

% Define ellipse (x-h)^2/r1^2 + (y-k)^2/r2^2 = 1
h = 1.45;
k = 0;
r1 = .5;
r2 = .5;

angle = [];

y_off = 0.0;
x_off = 0.0;

% A list of obstacles
obs_list = {[h k r1 r2]};
obs_list{end + 1} = [h k+1.5 r1 r2];
obs_list{end + 1} = [h k+-1.5 r1 + 0.0 r2];
obs_list{end + 1} = [h k r1 r2];
obs_list{end + 1} = [-1 -0 r1 r2];
% Define line segment that
% A,B,C of the line and end points (D,E) defined by the length of future
% path
line_list = {};
line_list{end + 1} = [A B 0 1 0];
line_list{end + 1} = [A B 0.15 1 -0.15];
line_list{end + 1} = [A B -.15 1 0.15];

h2 = figure(999);
plot_map(obs_list, line_list)
drawnow
%  Run GA algorithm
weight = [5 2.5 0.1]; % Optimization weights on Number of obstacle intersected, angle changed, and intersected length
N = 100;              % Number of candidates
T = 15;               % Number of parents candidates
sigma = .1;           % Pertubation Magnitude
params = [N T sigma];

% Find the best elite
elite = run_ga_multi_obstacle(weight, line_list, obs_list, params);
angle = [angle;elite];

% Plot the results
rot = [cos(elite) -sin(elite); sin(elite) cos(elite)];
new_vec = rot * [A;B];
C = -new_vec' * [0;0];
A = new_vec(1);
B = new_vec(2);

if A ~= 0 && B ~= 0
    x_line = linspace(-2+x_off,2+x_off,100);
    y_line = (-A * x_line - C) / B;
elseif A == 0 && B ~= 0
    x_line = linspace(-2+x_off,2+x_off,100);
    y_line = (-A * x_line - C) / B;
elseif A ~= 0 && B == 0
    y_line = linspace(-2,2,100);
    x_line = (-B * y_line - C) / A;
else
    error('not a valid line');
end
plot(x_line, y_line,'LineWidth',5);

legend('robot path','robot path','robot path','obstacle','obstacle','obstacle','obstacle','adjusted path')
