function [] = Casadi_Avoidance_Matrix_LeftStartV3(step_index)
import casadi.*
% System Parameters
Tstep = 0.4; % step time is 0.4, fixed for now
dt = 0.1; % MPC sample Time,
Npred = 4; % Number of steps
z0 = .9; % walking height
g = 9.81; % gravity term;
Nodes = Npred * round(Tstep / dt) + 1;

% Define Dynamics Matrix
A = [0 1;g/z0 0];
B = [0;-g/z0];
nx = size(A,2);
nu = size(B,2);

% Define MPC Variables
x = MX.sym('x',nx * Nodes); % x com traj
ux = MX.sym('ux',Nodes);   % x center of pressure traj
dPx = MX.sym('dPx',Npred); % x foot change
y = MX.sym('y',nx * Nodes); % y com traj
uy = MX.sym('uy',Nodes);   % y center of pressure traj
dPy = MX.sym('dPy',Npred); % y foot change
s = MX.sym('s',Nodes);     % environment slack variable
swf_q = MX.sym('swf_q', (round(Tstep / dt) + 1) * 3);      % swing foot var: x, y, z
swf_s = MX.sym('swf_s', round(Tstep / dt) + Npred);        % swing foot slack variable for x,y, and z
swf_bi = MX.sym('swf_bin',Npred * 4);                      % Binary variable for foothold selection
add_var = MX.sym('add_var', 7);                            % Distance Constraint for navigating around obstacle
avd_s = MX.sym('avd_s', 8);                                % x velocity tracking slack variable
Variable = [x;ux;dPx;y;uy;dPy;s;swf_q;swf_s;avd_s;add_var;swf_bi];

% Define MPC Parameters
q_init = MX.sym('q_init',nx * 2); % com current position
qo_ic = MX.sym('qo_ic',2);        % obstacle position that is cloeset to robot
qo_tan = MX.sym('qo_tan',2);      % norm vector of the obstacle point
x_ref = MX.sym('x_ref',Npred + 1);   % x reference
y_ref = MX.sym('y_ref',Npred + 1);   % y reference
f_length = MX.sym('f_length',2);  % Foot Length
f_init = MX.sym('f_init',2);      % Initial Foot Position
f_param = MX.sym('f_param',6);    % MPC Foot Related Parameters
Weights = MX.sym('Weights',9);    % MPC Weights
r = MX.sym('r',2);                % Obstacle Radius
rt = MX.sym('rt', 1);             % Remaining time of current trajectory
foff = MX.sym('foff', 2);         % Offset parameter for model gap
height = MX.sym('height', 2);     % Walking height and weight magnify factor for different step
swf_cq = MX.sym('swf_cq', 3);     % Current swing foot state, x
swf_rq = MX.sym('swf_rq', (round(Tstep / dt) + 1) * 3);      % Reference swing foot state
swf_Q = MX.sym('swf_Q', 3);       % Swing foot position tracking
swf_obs = MX.sym('swf_obs', 14);    % Foot Obstacle x,y,z, radius, weights, z fraction
add_param = MX.sym('add_param', 6); % Distance constraint for navigating obstacles

Input = [q_init;x_ref;y_ref;f_length;f_init;f_param;Weights;r;qo_ic;qo_tan;rt;foff;height...
    ;swf_cq;swf_rq;swf_Q;swf_obs;add_param];

A = [0 1;g/height(1) 0];
B = [0;-g/height(1)];

% Define Dynamics Constraint
eq_con = [];
for n = 1:Nodes
    %
    ux_n = ux(nu*(n-1)+1:nu*n) + foff(1); % Current foothold x
    x_n = x(nx*(n-1)+1:nx*n);             % Current CoM x
    uy_n = uy(nu*(n-1)+1:nu*n) + foff(2); % Current foothold y
    y_n = y(nx*(n-1)+1:nx*n);             % Current CoM y
    if n == 1
        eq_con = [eq_con;q_init(1:2,1) - x(1:nx,1);q_init(3:4,1) - y(1:nx,1)]; % Initial states
    else
        x_n1 = x(nx*(n-2)+1:nx*(n-1));               % Previous foothold x
        y_n1 = y(nx*(n-2)+1:nx*(n-1));               % Previous CoM x
        ux_n1 = ux(nu*(n-2)+1:nu*(n-1)) + foff(1);   % Previous foothold y
        uy_n1 = uy(nu*(n-2)+1:nu*(n-1)) + foff(2);   % Previous CoM y
        xdt = .5 * (A*x_n + B*ux_n) + .5 * (A*x_n1 + B*ux_n1);
        ydt = .5 * (A*y_n + B*uy_n) + .5 * (A*y_n1 + B*uy_n1);
        if n == 2
            eq_con = [eq_con;x_n - x_n1 - rt * xdt;y_n - y_n1 - rt * ydt];
        else
            eq_con = [eq_con;x_n - x_n1 - dt * xdt;y_n - y_n1 - dt * ydt];
        end
    end
end

% Initialize inequality Constraint
ieq_con = [];

% Define Avoidance Constraint
for n = 1:Nodes
    % TODO: make qo_ic a trajectory
    qn = [x(nx*(n-1)+1);y(nx*(n-1)+1)];
    % inner constraint
    inter = qo_ic + r(1) * qo_tan;
    ieq_in = -dot(qn - inter,qo_tan);
    % outer constraint
    outer = qo_ic + r(2) * qo_tan;
    ieq_out = -dot(qn - outer,qo_tan) - s(n);
    ieq_con = [ieq_con;ieq_in;ieq_out; -s(n)];
end

% Define Tracking Cost
cost = 0;
Qx = [Weights(1) 0;0 Weights(2)];
Qy = [Weights(3) 0;0 Weights(4)];
% Define position tracking during standing phase
x_c = (x_ref(1) - x(1:2:end)).' * Qx(1,1) * (x_ref(1) - x(1:2:end));
y_c = (y_ref(1) - y(1:2:end)).' * Qy(1,1) * (y_ref(1) - y(1:2:end));
cost = cost + x_c + y_c;

% Define velocity tracking for stepping phase
x_vel = x(2:2:end);
y_vel = y(2:2:end);
dx_e = x_vel(round(Tstep / dt)+1 - step_index:round(Tstep / dt):end) - x_ref(2:Npred + 1) - add_param(3) * add_var(5);
dy_e = y_vel(round(Tstep / dt)+1 - step_index:round(Tstep / dt):end) - y_ref(2:Npred + 1);

% Penalize initial velocity as well
if step_index < 3
    for i = 2:4-step_index
        dx_e = [dx_e; x_vel(i) - x_ref(2) - add_param(3) * add_var(5)];
    end
end
dx_e = [dx_e; x_vel(end) - x_ref(2)];

% Velocity tracking cost
ieq_con = [ieq_con; -avd_s(1:length(dx_e)) - dx_e;-avd_s(1:length(dx_e))  + dx_e];
cost = cost + avd_s.' * Qx(2,2) * avd_s + dy_e.' * Qy(2,2) * dy_e;

% Define Foot Placement Tracking Cost and Constraint (x direction is cost and y direction is constraint)
% TODO: Add slack variables
L = tril(ones(Npred)) - eye(Npred);
temp = [kron(L,ones(round(Tstep / dt),1));ones(1,Npred)];
temp = [temp(1 + step_index:end,:);ones(step_index,Npred)];

Ux_ref = ones(Nodes,1) * f_init(1) + temp * dPx;
Uy_ref = ones(Nodes,1) * f_init(2) + temp * dPy;

for n = 1:Nodes
    ux_e = Ux_ref(n) - ux(n);
    uy_e = Uy_ref(n) - uy(n);
    cost = cost + 1 * ux_e * Weights(5) * ux_e + 0 * uy_e * Weights(6) * uy_e;
end
eq_con = [eq_con;Uy_ref(:) - uy(:)];

% Foot length constraints
ieq_con = [ieq_con;(-1).^(2:Npred+1)' .* dPy + 0.1];
x_max_length = f_length(1) + abs(q_init(2)) * Tstep * 2;
% distance from COM to stance foot
ieq_con = [ieq_con;-.5 * x_max_length - x(1:2:end) + Ux_ref;x(1:2:end) - Ux_ref - .5 * x_max_length];
ieq_con = [ieq_con;-1 * f_length(2) - y(1:2:end) + Uy_ref;y(1:2:end) - Uy_ref - 1 * f_length(2)];
% distance from stance foot to swing foot
ieq_con = [ieq_con;-x_max_length - dPx;dPx - x_max_length];
ieq_con = [ieq_con;-f_length(2) - dPy;dPy - f_length(2)];

% a_max = add_param(5);
% dPx_last = add_param(6);
% t_left = rt + (3 - step_index) * 0.1;
% ieq_con = [ieq_con;-.5 * a_max * t_left^2 - (dPx(1) - dPx_last);(dPx(1) - dPx_last) - .5 * a_max * t_left^2];
% ieq_con = [ieq_con;-.5 * a_max * Tstep^2 - (dPx(1));(dPx(1)) - .5 * a_max * Tstep^2];

% Define Foot Change Cost
dix = f_param(3); % foot change ref in x direction
diy = f_param(6); % foot chagne ref in y

x_pos = x(1:2:end);
y_pos = y(1:2:end);
key_footy_pos = Uy_ref(round(Tstep/dt) + 1 -step_index:round(Tstep/dt):end,1);
key_y_pos = y_pos(round(Tstep/dt) + 1 -step_index:round(Tstep/dt):end,1);
key_footx_pos = Ux_ref(round(Tstep/dt) + 1 - step_index:round(Tstep/dt):end,1);
key_x_pos = x_pos(round(Tstep/dt) + 1 -step_index:round(Tstep/dt):end,1);

key_x_err = key_footx_pos - key_x_pos - dix;
key_y_err = key_footy_pos - key_y_pos - diy * (-1).^(1:Npred)';

cost = cost + key_x_err.' * Weights(7) * key_x_err + key_y_err.' * Weights(8) * key_y_err;

% Obstacle Slack Cost
for n = 1:Nodes
    cost = cost + s(n) * Weights(9) * s(n);
end

%% Additional costs and constraints for swing foot avoidance
% Swing Foot Trajectroy
dN = round(Tstep / dt) + 1;
swf_x = swf_q(1:dN);
swf_y = swf_q(dN + 1:2*dN);
swf_z = swf_q(2*dN+1:end);

% Reference Foot Trajectory
swf_rx = swf_rq(1:dN);
swf_ry = swf_rq(dN + 1:2*dN);
swf_rz = swf_rq(2*dN+1:end);

for n = step_index + 1 : round(Tstep / dt) + 1
    if n == step_index + 1
        % Initial State, Hard Constraint
        eq_con = [eq_con;swf_x(n) - swf_cq(1)...
            ;swf_y(n) - swf_cq(2)...
            ;swf_z(n) - swf_cq(3)];
    else
        % Final State, Hard Constraint
        if n == round(Tstep / dt) + 1
            eq_con = [eq_con;swf_x(n) - f_init(1) - dPx(1)...
                ;swf_y(n) - f_init(2) - dPy(1)...
                ;swf_z(n)];
        end
        % Internal State, Soft Constraint
        cost = cost + swf_Q(1) * (swf_x(n) - swf_rx(n)).^2 +...
            swf_Q(2) * (swf_y(n) - swf_ry(n)).^2 +...
            swf_Q(3) * (swf_z(n) - swf_rz(n)).^2;
    end
end

% Foot Avoidance part
swf_obs_pos = swf_obs(1:3,1);  % Foot obstacle position
swf_obs_r1 = swf_obs(4);       % x,y hard radius
swf_obs_r2 = swf_obs(5);       % x,y soft radius
swf_obs_r3 = swf_obs(6);       % z hard radius
swf_obs_r4 = swf_obs(7);       % z soft radius
swf_obs_Qxy = swf_obs(8);      % Avoidance cost in x,y
swf_obs_Qz = swf_obs(9);       % Avoidance cost in z
frac_z = swf_obs(10);          % Ratio between step height and height at mid step
min_z = swf_obs(11);           % Minimum step height
max_z = swf_obs(12);           % Maximum step height
M = swf_obs(13);               % Big M for mixed integer programming
th = swf_obs(14);              % theta for rotation quadrant

% Future foot position prediction
foot_vec = [];
foot_x = f_init(1) + dPx(1);
foot_y = f_init(2) + dPy(1);
for n = 2 : Npred
    foot_x = foot_x + dPx(n);
    foot_y = foot_y + dPy(n);
    foot_vec = [foot_vec;foot_x;foot_y];
end
rotA = [cos(th) -sin(th);sin(th) cos(th)];
rotB = [cos(th) sin(th);-sin(th) cos(th)];

for n = step_index + 1 : round(Tstep / dt) + 1
    % swing foot traj variable
    cur_p = [swf_x(n);swf_y(n);swf_z(n)];
    if n == step_index + 1
        ; % Can not change current position for avoidance
    elseif n == round(Tstep / dt) + 1
        % 2D avoidance for touchdown position (end of a step)
        for i = 1:Npred
            eq_con = [eq_con;sum(swf_bi(i * 4 - 3: i * 4)) - 1]; % Only one quadrant is selected
        end

        cp = cur_p(1:2);
        op = swf_obs_pos(1:2);
        
        cp = [cp;foot_vec];
        % Block one
        vec = rotB * [-1;1]/norm([-1;1]);
        inter = op + swf_obs_r1 * vec;
        inter2 = op + swf_obs_r2 * vec;
        ieq_con = [ieq_con;...
            cp(1) - op(1) - M*(1-swf_bi(1)) - swf_s(n) + swf_obs_r1 + 0.1;...
            -cp(2) + op(2) - M*(1-swf_bi(1)) - swf_s(n) + swf_obs_r1 - 0.1 ;...
            -dot(vec, cp(1:2) - inter) - M*(1-swf_bi(1))];

        for i = 0 : Npred - 1
            ieq_con = [ieq_con;-dot(vec, cp(2*i + 1: 2*i + 2) - inter2) - M*(1-swf_bi(1 + 4 * i)) - swf_s(n + i)];
        end

        % Block two
        vec = rotA * [-1;-1]/norm([-1;-1]);
        inter = op + swf_obs_r1 * vec;
        inter2 = op + swf_obs_r2 * vec;
        ieq_con = [ieq_con;...
            cp(1) - op(1) - M*(1-swf_bi(2)) - swf_s(n)  + swf_obs_r1 + 0.1;...
            cp(2) - op(2) - M*(1-swf_bi(2)) - swf_s(n) + swf_obs_r1  - 0.1;...
            -dot(vec, cp(1:2) - inter) - M*(1-swf_bi(2))];

        for i = 0 : Npred - 1
            ieq_con = [ieq_con;-dot(vec, cp(2*i + 1: 2*i + 2) - inter2) - M*(1-swf_bi(2 + 4 * i)) - swf_s(n + i)];
        end

        % Block three
        vec = rotA * [1;1]/norm([1;1]);
        inter = op + swf_obs_r1 * vec;
        inter2 = op + swf_obs_r2 * vec;
        ieq_con = [ieq_con;...
            -cp(1) + op(1) - M*(1-swf_bi(3)) - swf_s(n)  + swf_obs_r1 + 0.1;...
            -cp(2) + op(2) - M*(1-swf_bi(3)) - swf_s(n) + swf_obs_r1 - 0.1 ;...
            -dot(vec, cp(1:2) - inter) - M*(1-swf_bi(3))];

        for i = 0 : Npred - 1
            ieq_con = [ieq_con;-dot(vec, cp(2*i + 1: 2*i + 2) - inter2) - M*(1-swf_bi(3 + 4 * i)) - swf_s(n + i)];
        end

        % Block four
        vec = rotB * [1;-1]/norm([1;-1]);
        inter = op + swf_obs_r1 * vec;
        inter2 = op + swf_obs_r2 * vec;
        ieq_con = [ieq_con;...
            -cp(1) + op(1) - M*(1-swf_bi(4)) - swf_s(n)  + swf_obs_r1 + 0.1;...
            cp(2) - op(2) - M*(1-swf_bi(4)) - swf_s(n)  + swf_obs_r1 - 0.1 ;...
            -dot(vec, cp(1:2) - inter) - M*(1-swf_bi(4))];

        for i = 0 : Npred - 1
            ieq_con = [ieq_con;-dot(vec, cp(2*i + 1: 2*i + 2) - inter2) - M*(1-swf_bi(4 + 4 * i)) - swf_s(n + i)];
        end

        for i = 0 : Npred - 1
            cost = cost + swf_obs_Qxy * swf_s(n + i) * swf_s(n + i) * exp(height(2) * i);
        end
    else
        % Linearize the constraint
        % TODO: Modify this constraints
        ref_p = [swf_rx(n);swf_ry(n);swf_rz(n)];
        %vec = (ref_p - swf_obs_pos) / sqrt((ref_p - swf_obs_pos).' * (ref_p - swf_obs_pos));
        vec = (ref_p - swf_obs_pos) / norm_2(ref_p - swf_obs_pos);
        inter1 = swf_obs_pos + swf_obs_r3 * vec;
        inter2 = swf_obs_pos + swf_obs_r4 * vec;
        z1 = inter1(3);
        z2 = inter2(3);
        ieq_con = [ieq_con;-swf_z(n) + z2 - swf_s(n);-swf_z(n) + z1];
        cost = cost + swf_obs_Qz * swf_s(n) * swf_s(n);
    end
end
ieq_con = [ieq_con;-swf_s];


% symmetrical vertical trajectory
eq_con = [eq_con;swf_z(2) - swf_z(4)];            % symmetric foot trajectory 
ieq_con = [ieq_con;swf_z(2) - frac_z * swf_z(3)]; % foot height ratio
ieq_con = [ieq_con;swf_z(3) - max_z];             % max height
if step_index < 2
    ieq_con = [ieq_con;-swf_z(3) + min_z];        % min height
end

% Foot diff part    
x1 = add_var(1);
x2 = add_var(2);
y1 = add_var(3);
y2 = add_var(4);
s1 = add_var(5);
b1 = add_var(6);
b2 = add_var(7);

lb = add_param(1);          % minimum travel distance
Q = add_param(2);           % cost on travel distance soft constraint
signx = add_param(3);       % sign of walking direction x, + forward, - backward
signy = add_param(4);       % sign of walking direction y, + left, - right

cost = cost + Q * s1 * s1;
eq_con = [eq_con; x1 - (x_pos(end) - x_pos(1)); y1 - y2 - (y_pos(5) - y_pos(1))];
ieq_con = [ieq_con; -s1 + signx * (-x1 + lb) - abs(signx) * signy * (y1 + y2); -signx * x1];

ieq_con = [ieq_con;-y1;-y2;y1 - b2 * M;y2 - (1 - b2) * M];

% Get the jacobian and Hessian Information
Aeq = jacobian(eq_con,Variable);
beq = eq_con - Aeq * Variable;
Aiq = jacobian(ieq_con,Variable);
biq = ieq_con - Aiq * Variable;
H = hessian(cost,Variable);
f = jacobian(cost - .5 * Variable.' * H * Variable,Variable);

disp('generating c code')
fff = Function(char('LeftStart_Step' + string(step_index) + 'V3'),{Input,Variable},{Aeq,beq,Aiq,biq,H,f});
% opts = struct('mex',false,'main',false,'cpp',false,'with_header',false);
% func_name = char('RightStart_Step' + string(step_index));
% fff.generate(func_name,opts);
cg_options = struct('mex',true,'main',false,'cpp',false,'with_header',false);
cg = CodeGenerator(char('LeftStart_Step' + string(step_index) + 'V3'),cg_options);
cg.add(fff);
tic
cg.generate() 
toc

end

