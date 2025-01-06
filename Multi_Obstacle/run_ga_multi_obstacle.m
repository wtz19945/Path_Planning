function [elite] = run_ga_multi_obstacle(weight, line_list, obs_list, params)
%RUN_GA_MULTI_OBSTACLE Summary of this function goes here
%   Detailed explanation goes here
N = params(1);
T = params(2);
sigma = params(3);

P = pi / 2 * (rand(N,1) - .5);
newP = P;

F = zeros(N,1);

elite = 0;
last_elite = 0;

P = linspace(-pi/4, pi/ 4, N);
inter_length_vec = zeros(N,1);
for iter = 1: 100
    iter
    % Evaulate each parametrs
    for i = 1 : N
        inter_count = 0;
        inter_length = 0;
        rot = [cos(P(i)) -sin(P(i)); sin(P(i)) cos(P(i))];
        
        for j = 1: length(line_list)
            % Find number of intersections
            A_init = line_list{j}(1); 
            B_init = line_list{j}(2);
            C_init = line_list{j}(3);
            end_p_init = line_list{j}(4:5)';
            
            base_p = [-C_init/A_init;0];
            new_vec = rot * [A_init;B_init];
            new_C = -new_vec' * base_p;

            end_p = rot * (end_p_init-base_p) + base_p;

            for k = 1: length(obs_list)
                [intersect,inter_point] = find_intersect([new_vec(1),new_vec(2),new_C], obs_list{k});
                % Check the intersection length

                if intersect
                    inter_p1 = inter_point(1:2);
                    inter_p2 = inter_point(3:4);
                    
                    % rebase
                    inter_p1 = inter_p1 - base_p';
                    inter_p2 = inter_p2 - base_p';
                    end_p = end_p - base_p;

                    if dot(inter_p1, end_p) < 0 || dot(inter_p2, end_p) < 0
                        inter_count = inter_count + 0;
                        inter_length = inter_length + 0;
                    elseif max(abs(inter_p1(1)), abs(inter_p2(1))) <= abs(end_p(1))
                        inter_count = inter_count + 1;
                        inter_length = inter_length + norm(inter_p1 - inter_p2);
                    elseif min(abs(inter_p1(1)), abs(inter_p2(1))) >= abs(end_p(1))
                        inter_count = inter_count + 0;
                        inter_length = inter_length + 0;
                    else
                        inter_count = inter_count + 1;
                        if abs(inter_p1(1)) <= abs(inter_p2(1))
                            inter_length = inter_length + norm(inter_p1 - end_p');
                        else
                            inter_length = inter_length + norm(inter_p2 - end_p');
                        end
                    end
                end
            end
        end
        inter_length_vec(i) = inter_length;
        F(i) = weight * [inter_count;abs(P(i))^2;inter_length];
    end

    % figure
    % plot(P,F)
    % hold on
    % plot(P, weight(2) * abs(P).^2)
    % plot(P, weight(3) * inter_length_vec)
    % Rank the population
    [F, indices] = sort(F);
    P = P(indices);
    % Generate new population
    elite = P(1);
    newP(1) = elite;
    
    for i = 2 : N
        index = randi(T);
        newP(i) = min(max(P(index) + sigma * normrnd(0,1),pi/4), -pi/4);
    end

    P = newP;
    % early stop
    if norm(elite - last_elite) < 1e-6
        break;
    end
    last_elite = elite;
end

end

