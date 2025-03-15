%%
clear
clc
close all
% System Parameters
Tstep = 0.4;                     % step time is 0.4, fixed for now
dt = 0.1;                        % MPC sample Time, 
Npred = 4;                       % Number of steps
z0 = .9;                         % walking height
g = 9.81;                        % gravity term;
Nodes = Npred * round(Tstep / dt) + 1;
var_num = 181;

sim_step = 100;
stance_leg = 1;
n = 1;

q_init = [3;0.0;0;-0.06]; % intial robot state
dx_des = 0.3;
w = sqrt(9.81 / 0.9);
x0_ref = dx_des / w * (2 - exp(w * Tstep) - exp(-w * Tstep)) / (exp(w * Tstep) - exp(-w * Tstep));
dy_des = sqrt(w) * 0.11 * tanh(sqrt(w) * Tstep/2);
dy_off = 0.0;
x_ref = dx_des * ones(Npred + 1,1);
f_length = [.8;.4];     % foot length limit
f_param = [0;0;x0_ref;0;0;0.1]; % foot parameters
Weights = [0;5000;0;5000;100000;-1000000;500;500;5000];  % MPC weights
add_param = [Tstep * dx_des * 4 + 0.4;1000;1;0.0;5;0]; % Currently index5,6 is not used

r = [0.1;0.5];                          % obstacle radius
qo_ic = [-10.5;0.0];                    % obstacle position
qo_tan = [q_init(1);q_init(3)] - qo_ic; % obstacle tangent vector

[sx,sy,sz] = sphere;

qo_tan = qo_tan/norm(qo_tan);
du_ref = [0.9;0];
swf_Q = 10*[10;10;100];

swf_obs_pos = {[2.35;0.1;0.0]};    % Foot obstacle position
swf_obs_pos{end+1} = [2.35;0.5;0.0];    % Foot obstacle position
swf_obs_pos{end+1} = [2.35;-0.6;0.0];    % Foot obstacle position
swf_obs_pos{end+1} = [1.35;0.1;0.0];    % Foot obstacle position

swf_obs_size = {[0.3;0.4]};
swf_obs_size{end+1} = [.3;.4]; 
swf_obs_size{end+1} = [.3;.4];
swf_obs_size{end+1} = [.2;.3];

swf_xy_r = swf_obs_size{1};      % x,y safe radius
swf_xy_z = [0.0;0.3];            % z safe radius
swf_Q_soft = [8000;8000];        % Avoidance cost in x-y and z
frac_z = 0.5;                    % step height fraction
M = 10000;                       % big M for mixed integer programming
th = 0;

swf_obs = [swf_obs_pos{1};swf_xy_r;swf_xy_z;swf_Q_soft;frac_z;0.2;0.6;M;th];
swf_obs_info = [swf_obs(1:3);swf_xy_r(1)];
x_goal = 0.0 + 0.5 * 0.4;
y_goal = 0.11;
z_goal = [0.0;0.1;0.2;0.1;0.0];


% Set initial state
fx_start = 0;
fy_start = 0.11;
swf_cq = [0;0.11;0.0];
swf_rq = [linspace(fx_start, x_goal, 5)'; linspace(fy_start, y_goal, 5)'; z_goal];

q_init = [0;-0.0;0;-0.06]; % intial robot state
y_ref = [0;dy_off + dy_des * (-1).^(2:Npred + 1).'];
f_init = [0;-0.11];      % foot initial state


% Obstacle
angle = 0:0.1:2*pi;
xoff = sin(angle);
yoff = cos(angle);

view_opt = 1;
if view_opt == 1
    filename = 'Walking0.gif'; % Name of the GIF file
elseif view_opt == 2
    filename = 'Walking1.gif'; % Name of the GIF file
else
    filename = 'no_avoidance.gif'; % Name of the GIF file
end

h = figure;
delayTime = 0.1;


z_traj = [];
solve_time = [];
distance = [];
speedup_flag = [];
slack_var = [];
design_vector = [];

elite_vec = [];
elite = 0;
x_ref_off = 0;
y_ref_off = 0;

% Initialize QP
for i = 1:sim_step
    % Find the cut
    Aiq_extra = [];
    biq_extra = [];
    if i > 2
        x_pos = design_vector(1:2:Nodes * 2);
        y_pos = design_vector(3*Nodes + Npred + 1:2:5*Nodes + Npred);

        % Compute A, B, C the line
        start_point = [x_pos(1);y_pos(1)];
        end_point = [x_pos(end) + 1;y_pos(1)] - start_point;

        % Compute list of obstacles
        obs_list = {};
        for j = 1:length(swf_obs_pos)
            obs_list{end + 1} = [swf_obs_pos{j}(1) - start_point(1)  swf_obs_pos{j}(2) - start_point(2)...
                 swf_obs_size{j}(1) swf_obs_size{j}(1)];
        end

        % Compute list of line boundaries
        start_point = zeros(2,1);
        point_vec = end_point - start_point;
        theta = 90; % to rotate 90 counterclockwise
        R = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
        norm_vec = R * point_vec;
        A = norm_vec(1);
        B = norm_vec(2);
        C = -norm_vec' * start_point;
        line_list = {};
        line_list{end + 1} = [A B C end_point(1) end_point(2)];
        line_list{end + 1} = [A B -norm_vec' * [0.0;0.15] end_point(1) end_point(2) + 0.15];
        line_list{end + 1} = [A B -norm_vec' * [0.0;-0.15] end_point(1) end_point(2) - 0.15];

        % Plot the figure
        h2 = figure(2);  % Use a unique handle for the new figure
        clf(h2);  % Clear the figure to avoid overlap

        weight = [1 2.5 0.1]; % Optimization weights on Number of obstacle intersected, angle changed, and intersected length
        N = 100;              % Number of candidates
        T = 15;               % Number of parents candidates
        sigma = .1;           % Pertubation Magnitude
        elite_guess = 0;      % Intial guess
        min_len = 0.4;        % Minimum obstacle length to be considered as partial intersection
        max_len = 0.6;        % obstacle length to be considered as full intersection
        params = [N T sigma min_len max_len];
        plot_map(obs_list, line_list)  
        [elite, obs_index] = run_ga_multi_obstacle(weight, line_list, obs_list, params, elite_guess);
        elite_vec = [elite_vec;elite];
        
        plot(x_pos- x_pos(1), y_pos- y_pos(1))

        % Plot the results
        rot = [cos(elite) -sin(elite); sin(elite) cos(elite)];
        new_vec = rot * [A;B];
        C = -new_vec' * [0;0];
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
        plot(x_line, y_line,'LineWidth',5);

        drawnow
        title('Additional Figure, Step ');

        % Adjust the quadrant and half-space
        if obs_index == 0
            % no obstacle is around
        else
            swf_obs(1:5) = [swf_obs_pos{obs_index};swf_obs_size{obs_index}];
            plot(obs_list{obs_index}(1),obs_list{obs_index}(2),'r*')
        end
        
        % Adjust the reference
        y_ref_off = sin(elite) * dx_des;
        x_ref = cos(elite) * dx_des * ones(Npred + 1,1);

        % Add cut plane
        if obs_index == 0
            % no obstacle is around
        else
            tar_obs = swf_obs_pos{obs_index};
            for j = 1:length(swf_obs_pos)
                nxt_obs = swf_obs_pos{j};
                if tar_obs(1) + swf_obs_size{obs_index}(1) > nxt_obs(1) - swf_obs_size{j}(1) && j~=obs_index
                    dPy = design_vector(6*Nodes + Npred + 1:6*Nodes + Npred + Npred);
                    actual_foot_y = [f_init(2);f_init(2) + cumsum(dPy)];
                    pos_diff = nxt_obs(2) - tar_obs(2);
                    Aiq_extra = [Aiq_extra;zeros(1,6*Nodes + Npred) sign(pos_diff) zeros(1, var_num - (6*Nodes + Npred + 1))];
                    biq_extra = [biq_extra;sign(pos_diff) * (nxt_obs(2) - f_init(2))];
                    if j == 3
                        biq_extra(end) = biq_extra(end) - swf_obs_size{j}(1);
                    end
                end
            end
        end
    end
    


    if stance_leg == 1
        y_ref = [0;dy_off + dy_des * (-1).^(2:Npred + 1).'] + y_ref_off;
        switch n
            case 1
                Input = [q_init;x_ref;y_ref;f_length;f_init;f_param;Weights;r;qo_ic;qo_tan;0.1;0;0;du_ref;swf_cq;swf_rq;swf_Q;swf_obs;add_param];
                [a,b,c,d,e,f] = RightStart_Step0V3(Input,0*rand(var_num,1));
            case 2
                Input = [q_init;x_ref;y_ref;f_length;f_init;f_param;Weights;r;qo_ic;qo_tan;0.1;0;0;du_ref;swf_cq;swf_rq;swf_Q;swf_obs;add_param];
                [a,b,c,d,e,f] = RightStart_Step1V3(Input,0*rand(var_num,1));
            case 3
                Input = [q_init;x_ref;y_ref;f_length;f_init;f_param;Weights;r;qo_ic;qo_tan;0.1;0;0;du_ref;swf_cq;swf_rq;swf_Q;swf_obs;add_param];
                [a,b,c,d,e,f] = RightStart_Step2V3(Input,0*rand(var_num,1));
            case 4
                Input = [q_init;x_ref;y_ref;f_length;f_init;f_param;Weights;r;qo_ic;qo_tan;0.1;0;0;du_ref;swf_cq;swf_rq;swf_Q;swf_obs;add_param];
                [a,b,c,d,e,f] = RightStart_Step3V3(Input,0*rand(var_num,1));
            otherwise
                pause;
        end
    else
        y_ref = [0;dy_off + dy_des * (-1).^(1:Npred).'] + y_ref_off; 
        switch n
            case 1
                Input = [q_init;x_ref;y_ref;f_length;f_init;f_param;Weights;r;qo_ic;qo_tan;0.1;0;0;du_ref;swf_cq;swf_rq;swf_Q;swf_obs;add_param];
                [a,b,c,d,e,f] = LeftStart_Step0V3(Input,0*rand(var_num,1));
            case 2
                Input = [q_init;x_ref;y_ref;f_length;f_init;f_param;Weights;r;qo_ic;qo_tan;0.1;0;0;du_ref;swf_cq;swf_rq;swf_Q;swf_obs;add_param];
                [a,b,c,d,e,f] = LeftStart_Step1V3(Input,0*rand(var_num,1));
            case 3
                Input = [q_init;x_ref;y_ref;f_length;f_init;f_param;Weights;r;qo_ic;qo_tan;0.1;0;0;du_ref;swf_cq;swf_rq;swf_Q;swf_obs;add_param];
                [a,b,c,d,e,f] = LeftStart_Step2V3(Input,0*rand(var_num,1));
            case 4
                Input = [q_init;x_ref;y_ref;f_length;f_init;f_param;Weights;r;qo_ic;qo_tan;0.1;0;0;du_ref;swf_cq;swf_rq;swf_Q;swf_obs;add_param];
                [a,b,c,d,e,f] = LeftStart_Step3V3(Input,0*rand(var_num,1));
            otherwise
                pause;
        end
        
    end

    Aeq_dense = full(a);
    beq_dense = -full(b);
    Aiq_dense = full(c);
    biq_dense = -full(d);
    H_dense = full(e);
    f_dense = full(f);
    
    Aiq_dense = [Aiq_dense;Aiq_extra];
    biq_dense = [biq_dense;biq_extra];

    var_num = size(Aeq_dense,2);
    Aiq_num = size(Aiq_dense,1);
    Aeq_num = size(Aeq_dense,1);
    
    var_num
    Aiq_num + Aeq_num
    vartype = [repmat('C',var_num - Npred * 4 - 2, 1); repmat('B', Npred * 4 + 2, 1)]; % Define variables
    lb = -Inf * ones(var_num, 1);
    ub = Inf * ones(var_num, 1);
    model.Q = sparse(.5*H_dense);
    model.obj = f_dense;
    model.A = sparse([Aiq_dense;Aeq_dense]);
    model.rhs = [biq_dense;beq_dense];
    model.sense = [repmat('<',Aiq_num,1); repmat('=',Aeq_num,1)];
    model.vtype = vartype;
    model.lb = lb;
    model.ub = ub;
    model.modelsense = 'min';
    if i > 1
        % warm start
        model.start = initial_guess;
    end

    % Tune MIQP
    gurobiParams = struct();
    gurobiParams.TimeLimit = 3600;        % Time limit of 1 hour
    %gurobiParams.MIPGap = 1e-4;           % 1% MIP gap
    %gurobiParams.Presolve = 2;            % Aggressive presolve
    %gurobiParams.Cuts = 2;                % Aggressive cut generation
    %gurobiParams.Threads = 16;             % Use 4 threads
    %gurobiParams.MIPFocus = 1;            % Focus on finding feasible solutions
    %gurobiParams.Heuristics = 0.05;        % Increase heuristics effort
    %gurobiParams.FeasibilityTol = 1e-5;
    %gurobiParams.OptimalityTol = 1e-5;3
    % Set Gurobi parameters (optional)
    gurobiParams.outputflag = 0; % Display output
    
    % Solve the MIQP problem
    result = gurobi(model, gurobiParams);
    design_vector = result.x;
    initial_guess = design_vector;
    solve_time = [solve_time;result.runtime];
    fprintf('Gurobi run time is: %f seconds\n', result.runtime);
    
    % Unpack the solution
    traj_x = design_vector(1:Nodes * 2);
    traj_y = design_vector(3*Nodes + Npred + 1:5*Nodes + Npred);
    
    dPx = design_vector(3*Nodes + 1:3*Nodes + Npred);
    dPy = design_vector(6*Nodes + Npred + 1:6*Nodes + Npred + Npred);
    
    add_param(6) = dPx(1);
    actual_foot_x = [f_init(1);f_init(1) + cumsum(dPx)];
    actual_foot_y = [f_init(2);f_init(2) + cumsum(dPy)];
    
    swing_foot = design_vector(7*Nodes + 2 * Npred + 1 : 7*Nodes + 2 * Npred + 15);
    swing_foot_x = swing_foot(1:5);
    swing_foot_y = swing_foot(6:10);
    swing_foot_z = swing_foot(11:15);
    
    % Plot Data
    figure(h);
    plot3(traj_x(1:2:end),traj_y(1:2:end),ones(size(traj_x(1:2:end))))
    if view_opt == 1
        view(15,90);
    elseif view_opt == 2
        view(45,135);
    else
        view(15,90);
    end
    hold on
    plot3(traj_x(1),traj_y(1), 1,'o','MarkerSize',5,    'MarkerEdgeColor','b',...
        'MarkerFaceColor','y')
    plot(actual_foot_x(1),actual_foot_y(1),'o','MarkerSize',5,    'MarkerEdgeColor','b',...
        'MarkerFaceColor','r')
    plot(actual_foot_x(2),actual_foot_y(2),'o','MarkerSize',10,    'MarkerEdgeColor','b',...
        'MarkerFaceColor','g')
    plot3(swf_cq(1),swf_cq(2), swf_cq(3),'o','MarkerSize',5,    'MarkerEdgeColor','b',...
        'MarkerFaceColor','b')
    plot3([traj_x(1) actual_foot_x(1)],[traj_y(1) actual_foot_y(1)], [1 0])
    plot3([traj_x(1) swf_cq(1)],[traj_y(1) swf_cq(2)], [1 swf_cq(3)])
    
    for j = 3 : Npred 
        plot(actual_foot_x(j),actual_foot_y(j),'o','MarkerSize',10,    'MarkerEdgeColor','b',...
        'MarkerFaceColor','g')
    end

    for j = 1:length(swf_obs_pos)
        cur_size = swf_obs_size{j}(1);
        hSurface = surf(cur_size * sx + swf_obs_pos{j}(1), cur_size* sy + swf_obs_pos{j}(2),...
            cur_size* sz + swf_obs_pos{j}(3), 'FaceColor', 'red');
        set(hSurface,'FaceColor',[0 0 1], ...
            'FaceAlpha',1,'FaceLighting','gouraud')
    end
    speedup_flag = [speedup_flag;design_vector(end - 4 * Npred - 2)];
    slack_var = [slack_var; sum(design_vector(end - 4 * Npred - 18: end - 4 * Npred - 15))];
    bin = design_vector(end- 4 * Npred + 1:end-4 * Npred + 4);
    
    % Plot the selected region
    index = find(bin > 0.5);
    ox = swf_obs(1);
    width = 2;
    oy = swf_obs(2);
    height = 2;
    switch index
        case 3
            fill([ox + height, ox, ox, ox + height], [oy, oy, oy + width, oy + width], 'r', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
        case 4
            fill([ox + height, ox, ox, ox + height], [oy, oy, oy - width, oy - width], 'g', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
        case 1
            fill([ox - height, ox, ox, ox - height], [oy, oy, oy + width, oy + width], 'b', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
        case 2
            fill([ox - height, ox, ox, ox - height], [oy, oy, oy - width, oy - width], 'y', 'FaceAlpha', 0.5, 'EdgeColor', 'none');
    end

    title(n)
    hold off

    xlim([-0.2 4])
    ylim([-.8 0.8])
    zlim([0 1.2])
    legend('Traj Ref', 'Base', 'St Foot', 'Sw Foot Des', 'Sw Foot Cur', 'St Leg', 'Sw Leg')
    xlabel('x')
    ylabel('y')
    % Capture the plot as an image
    frame = getframe(h);
    im = frame2im(frame);
    [imind, cm] = rgb2ind(im, 256);

    % Write to the GIF file
    if i == 1
        imwrite(imind, cm, filename, 'gif', 'Loopcount', inf, 'DelayTime', delayTime);
    else
        imwrite(imind, cm, filename, 'gif', 'WriteMode', 'append', 'DelayTime', delayTime);
    end
    
    distance = [distance;norm([q_init(1);q_init(3)] - swf_obs_info(1:2))];
    
    % Reset State
    q_init = [traj_x(3:4);traj_y(3:4)];
    x_goal = actual_foot_x(2);
    y_goal = actual_foot_y(2);
    z_goal = [z_goal(1:n);swing_foot_z(n+1:end)];

    n = n+1;
    z_traj = [z_traj;swf_cq(3)];
    swf_cq = [swing_foot_x(n); swing_foot_y(n); swing_foot_z(n)];
    swf_rq = [linspace(fx_start, x_goal, 5)'; linspace(fy_start, y_goal, 5)'; z_goal];

    if n > 4
        fx_start = f_init(1);
        fy_start = f_init(2);
        z_goal = [0.0;0.1;0.2;0.1;0.0];
        swf_rq = [linspace(fx_start, actual_foot_x(3), 5)'; linspace(fy_start, actual_foot_y(3), 5)'; z_goal];
        swf_cq = [fx_start; fy_start; 0];
        f_init = [actual_foot_x(2);actual_foot_y(2)];
        stance_leg = stance_leg * -1;
        n = 1;
    end
    
end

figure
plot(solve_time)
title('solve time')

figure
plot(speedup_flag);
title('speedup_flag')

figure
plot(slack_var)
hold on
plot(distance);
