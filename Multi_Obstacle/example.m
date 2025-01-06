clear
close all;
clc


% Define line
A = 1; 
B = 0;
C = 1;

% Define ellipse
h = 0;
k = 1;
r1 = 2;
r2 = 2;

% Rotation
theta = pi/6;
rot = [cos(theta) -sin(theta); sin(theta) cos(theta)];
new_vec = rot * [A;B];
C = new_vec' * [C/A;0];
A = new_vec(1);
B = new_vec(2);

if A ~= 0 && B ~= 0
    x_line = linspace(-2,2,100);
    y_line = (-A * x_line - C) / B;
elseif A == 0 && B ~= 0
    x_line = linspace(-2,2,100);
    y_line = (-A * x_line - C) / B;
elseif A ~= 0 && B == 0
    y_line = linspace(-2,2,100);
    x_line = (-B * y_line - C) / A;
else
    error('not a valid line');
end


theta = linspace(-pi, pi, 100);
x_elip = h + r1 * cos(theta);
y_elip = k + r2 * sin(theta);

% find intersect
[intersect,inter_point] = find_intersect([A,B,C], [h,k,r1,r2]);

% Plot 
figure
plot(x_line, y_line);
hold on
plot(x_elip, y_elip);
if intersect
    x1 = inter_point(1);
    y1 = inter_point(2);
    x2 = inter_point(3);
    y2 = inter_point(4);

    plot(x1,y1,'o','MarkerSize',10);
    plot(x2,y2,'o','MarkerSize',10);
end
plot(-1,0,'o','MarkerSize',10);
xlim([-4 4])
ylim([-4 4])

