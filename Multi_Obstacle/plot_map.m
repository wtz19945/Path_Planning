function [] = plot_map(obs_list, line_list)
%PLOT_MAP Summary of this function goes here
%   Detailed explanation goes here

figure
hold on

for i = 1:length(line_list)
    A = line_list{i}(1); 
    B = line_list{i}(2);
    C = line_list{i}(3);

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
    
    plot(x_line, y_line);

end



% Plot ellipse
for i = 1 : length(obs_list)
    h = obs_list{i}(1);
    k = obs_list{i}(2);
    r1 = obs_list{i}(3);
    r2 = obs_list{i}(4);
    theta = linspace(-pi, pi, 100);
    x_elip = h + r1 * cos(theta);
    y_elip = k + r2 * sin(theta);
    plot(x_elip, y_elip);
end


xlim([-4 4])
ylim([-4 4])

end

