%% 2D Obstacle Avoidance QP Matrix Formulation
clear  
clc

import casadi.*
% System Parameters
Tstep = 0.3; % step time is 0.4, fixed for now
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

% Define Variables
x = MX.sym('x',nx * Nodes); % x com traj
ux = MX.sym('ux',Nodes);   % x center of pressure traj 
dPx = MX.sym('dPx',Npred); % x foot change
y = MX.sym('y',nx * Nodes); % y com traj 
uy = MX.sym('uy',Nodes);   % y center of pressure traj 
dPy = MX.sym('dPy',Npred); % y foot change
s = MX.sym('s',Nodes);     % slack variable
s2 = MX.sym('s2',Nodes);   % slack variable

Variable = [x;ux;dPx;y;uy;dPy;s;s2];

% Define Parameters
q_init = MX.sym('q_init',nx * 2); % com current position 
qo_ic = MX.sym('qo_ic',2);        % obstacle position that is cloeset to robot
qo_tan = MX.sym('qo_tan',2);      % norm vector of the obstacle point
q_ref = MX.sym('q_ref',nx * 2);   % com reference
f_length = MX.sym('f_length',2);  % Foot Length
f_init = MX.sym('f_init',2);      % Initial Foot Position
f_param = MX.sym('f_param',6);    % MPC Foot Related Parameters
Weights = MX.sym('Weights',9);    % MPC Weights
r = MX.sym('r',2);                % Obstacle Radius

Input = [q_init;qo_ic;q_ref;f_length;f_init;f_param;Weights;r];

% Define Dynamics Constraint
eq_con = [];
for n = 1:Nodes
    %
    ux_n = ux(nu*(n-1)+1:nu*n);
    x_n = x(nx*(n-1)+1:nx*n);
    uy_n = uy(nu*(n-1)+1:nu*n);
    y_n = y(nx*(n-1)+1:nx*n);
    if n == 1
        eq_con = [eq_con;q_init(1:2,1) - x(1:nx,1);q_init(3:4,1) - y(1:nx,1)]; % Initial states
    else
        x_n1 = x(nx*(n-2)+1:nx*(n-1));
        y_n1 = y(nx*(n-2)+1:nx*(n-1));
        eq_con = [eq_con;x_n - x_n1 - dt * (A*x_n1 + B*ux_n);y_n - y_n1 - dt * (A*y_n1 + B*uy_n)]; 
    end
end

% Define Reachability Constraint
ieq_con = [];
% TODO: make f_length based on foot velocity and stepping time
ieq_con = [ieq_con;-f_length(1) - x(1:2:end) + ux;x(1:2:end) - ux - f_length(1)]; 
ieq_con = [ieq_con;-f_length(2) - y(1:2:end) + uy;y(1:2:end) - uy - f_length(2)]; 

% Define Avoidance Constraint
for n = 1:Nodes
    % TODO: make qo_ic a trajectory
    qn = [x(nx*(n-1)+1);y(nx*(n-1)+1)];
    % inner constraint
    inter = qo_ic + r(1) * qo_tan;
    ieq_in = -dot(qn - inter,qo_tan);
    % outer constraint
    outer = qo_ic + r(2) * qo_tan;
    ieq_out = -dot(qn - inter,qo_tan) - s(n);
    ieq_con = [ieq_con;ieq_in;ieq_out];
end

% Define Tracking Cost
cost = 0;
Qx = [Weights(1) 0;0 Weights(2)];
Qy = [Weights(3) 0;0 Weights(4)];
for n = 1:Nodes
    x_e = x(nx*(n-1)+1:nx*n) - q_ref(1:2,1);
    y_e = y(nx*(n-1)+1:nx*n) - q_ref(3:4,1);
    cost = cost + x_e.' * Qx * x_e + y_e.' * Qy * y_e;
end

% Define Foot Placement Tracking Cost and Constraint, Start with a simple
% case
step_index = 0;
L = tril(ones(Npred)) - eye(Npred);
temp = [kron(L,ones(round(Tstep/dt),1));ones(1,Npred)];
temp = [temp(1 + step_index:end,:);ones(step_index,Npred)];

Ux_ref = ones(Nodes,1) * f_init(1) + temp * dPx;
Uy_ref = ones(Nodes,1) * f_init(2) + temp * dPy;

for n = 1:Nodes
    ux_e = Ux_ref(n) - ux(n);
    uy_e = Uy_ref(n) - uy(n);
    cost = cost + ux_e * Weights(5) * ux_e + uy_e * Weights(6) * uy_e;
end

% Define Foot Change Cost
dix = f_param(3);
diy = f_param(6);

y_pos = y(1:2:end);
for n = 1:Npred
    dP_ex = dPx(n) - dix;
    if n < Npred
        start_ind = n*floor(Tstep/dt) + 1-step_index;
        end_ind = (n+1)*floor(Tstep/dt)-step_index;
    else
        start_ind = n*floor(Tstep/dt) + 1-step_index;
        end_ind = Nodes;
    end
    FB_dis = Uy_ref(start_ind:end_ind,1) - y_pos(start_ind:end_ind,1);
    s2_n = s2(start_ind:end_ind,1);
    ieq_con = [ieq_con;(-1)^(n+1) * (FB_dis - s2_n) + diy];
    cost = cost + dP_ex * Weights(7) * dP_ex;
end

for n = 1:Nodes
    cost = cost + s2(n) * Weights(8) * s2(n);
end

% Obstacle Slack Cost
for n = 1:Nodes
    cost = cost + s(n) * Weights(9) * s(n);
end

% Get jacobian and Hessian Information
Aeq = jacobian(eq_con,Variable);
beq = eq_con - Aeq * Variable;
Aiq = jacobian(ieq_con,Variable);
biq = ieq_con - Aiq * Variable;
H = hessian(cost,Variable);
f = jacobian(cost - .5 * Variable.' * H * Variable,Variable);

disp('generating cpp code')
fff = Function('fff',{Input,Variable},{Aeq,beq,Aiq,biq,H,f});
opts = struct('mex',true,'main',true,'cpp',true,'with_header',true);
fff.generate('gen',opts);

% disp('generating mex code')
% tic
% mex gen.cpp -largeArrayDims %
% toc
%%
q_init = [0;0.0;0;0.0];
q_ref = [0;0.6;0;0.0];
f_length = [.8;.4];
f_init = [0;0.07];
f_param = [0;0;.6 * .4;0;0;0.14];
Weights = [0;2500;0;2500;10000;10000;100;6000;15000];
r = [0.2;0.4];
qo_ic = [1.8;0.1];
Input = [q_init;qo_ic;q_ref;f_length;f_init;f_param;Weights;r];

[a,b,c,d,e,f] = gen(Input,0*rand(144,1));

[a1,b1,c1,d1,e1,f1] = gen(Input,2.5*rand(144,1));

ea = norm(full(a1) - full(a))
eb = norm(full(b1) - full(b))
ec = norm(full(c1) - full(c))
ed = norm(full(d1) - full(d))
ee = norm(full(e1) - full(e))
ef = norm(full(f1) - full(f))
%%
Aeq_dense = full(a);
beq_dense = full(b);
Aiq_dense = full(c);
biq_dense = -full(d);
H_dense = full(e);
f_dense = full(f);

% Variable = [x;ux;dPx;y;uy;dPy;s;s2];
% Input = [q_init;qo_ic;q_ref;f_length;f_init;f_param;Weights;r];
prob = osqp;
setup(prob,H_dense, f_dense, [Aiq_dense;Aeq_dense], [-inf*ones(size(Aiq_dense,1),1);beq_dense], [biq_dense;beq_dense]);
[design_vector,flag,dual] = prob.solve();

x = design_vector(1:2:32);
y = design_vector(56:2:87);
figure
plot(x,y)