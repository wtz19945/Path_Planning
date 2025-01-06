clear
close all;
clc


% Define line
A = 1; 
B = -0;
C = 0;

% Define ellipse
h = 0;
k = 2;
r1 = .5;
r2 = .5;

angle = [];
y_off_vec = [];

for i = 1:20
y_off = -1 + .1 * i;
y_off_vec = [y_off_vec;y_off];
x_off = 0;

obs_list = {[h+x_off k-y_off r1 r2]};
obs_list{end + 1} = [-1.5+x_off k-y_off r1 r2];
obs_list{end + 1} = [-.8 .75-y_off r1 + 0.0 r2];
obs_list{end + 1} = [1.5+x_off k-y_off r1 r2];

% Define line segment
line_list = {};
line_list{end + 1} = [1 0 0-x_off 0+x_off 1];
line_list{end + 1} = [1 0 0.15-x_off -0.15+x_off 1];
line_list{end + 1} = [1 0 -.15-x_off .15+x_off 1];

% plot_map(obs_list, line_list)

%  Run GA algorithm
weight = [1 2.5 0.1];
N = 100;
T = 15;
sigma = .1;
params = [N T sigma];

elite = run_ga_multi_obstacle(weight, line_list, obs_list, params);
angle = [angle;elite];
% 
% rot = [cos(elite) -sin(elite); sin(elite) cos(elite)];
% new_vec = rot * [A;B];
% A = new_vec(1);
% B = new_vec(2);
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
% plot(x_line, y_line);
end

figure
plot(y_off_vec, angle)

%%
clear
close all;
clc


% Define line
A = 1; 
B = -0;
C = 0;

% Define ellipse
h = 0;
k = 2;
r1 = .5;
r2 = .5;

angle = [];

y_off = .51;
x_off = 0.0;

obs_list = {[h k-y_off r1 r2]};
obs_list{end + 1} = [-1.5 k-y_off r1 r2];
obs_list{end + 1} = [-.8 .75-y_off r1 + 0.0 r2];
obs_list{end + 1} = [1.5 k-y_off r1 r2];

% Define line segment
line_list = {};
line_list{end + 1} = [1 0 0-x_off 0+x_off 1];
line_list{end + 1} = [1 0 0.15-x_off -0.15+x_off 1];
line_list{end + 1} = [1 0 -.15-x_off .15+x_off 1];

plot_map(obs_list, line_list)

%  Run GA algorithm
weight = [1 2.5 0.1];
N = 100;
T = 15;
sigma = .1;
params = [N T sigma];

elite = run_ga_multi_obstacle(weight, line_list, obs_list, params);
angle = [angle;elite];
% 
rot = [cos(elite) -sin(elite); sin(elite) cos(elite)];
new_vec = rot * [A;B];
C = -new_vec' * [(C+x_off)/A;0];
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
