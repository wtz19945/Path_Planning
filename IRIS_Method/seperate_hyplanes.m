function [Afinal,bfinal, pfinal] = seperate_hyplanes(C, d, obs_list)
%SEPERATE_HYPLANES Summary of this function goes here
%   Detailed explanation goes here
%   C, d: Parametrization of ellipse
%   obs_list: list of obstacles

Obs_checked = false(size(obs_list));
Obs_remain = true(size(obs_list));


Afinal = [];
bfinal = [];
pfinal = [];
while any(Obs_remain)
    % Find cloest obstacle lstar
    valid_idx = find(Obs_remain == true);
    min_dist = inf;
    min_idx = 0;
    for idx = valid_idx
        dis = vecnorm(obs_list{idx} - d', 2, 2);
        [value, ~] = min(dis);
        if value < min_dist
            min_idx = idx;
            min_dist = value;
        end
    end
    lstart = obs_list{min_idx};

    % Find the cloest point on lstart
    Aeq = zeros(size(lstart))';
    [row, col] = size(lstart);
    for n = 1:size(Aeq,2)
        Aeq(:,n) = C \ (lstart(n,:)' - d);
    end
    H = blkdiag(eye(col), zeros(row,row));
    f = zeros(col + row, 1);
    A = [];
    b = [];
    Aeq = [eye(col) -Aeq; zeros(1,col) ones(1, row)];
    beq = [zeros(col,1);1];
    lb = [-inf * ones(col,1);zeros(row,1)];
    ub = [];

    sol = quadprog(H,f,A,b,Aeq,beq,lb,ub);
    p = sol(1:col);

    % Reconstruct points
    xstar = C * p + d;


    % Compute the tangent plane
    norm_vec = 2 * (C \ ((C') \ (xstar - d)));
    norm_vec = norm_vec / norm(norm_vec);
    b = norm_vec' * xstar;
    if norm_vec' * d - b > 0 
        norm_vec = -norm_vec;
    end


    % Do pruning
    for idx = valid_idx
        obs = obs_list{idx};
        temp = obs * norm_vec - b;
        if min(temp) >= -1e-3
            Obs_remain(idx) = false;
            Obs_checked(idx) = true;
        end
    end

    Afinal = [Afinal; norm_vec'];
    bfinal = [bfinal;b];
    pfinal = [pfinal;xstar'];
end


