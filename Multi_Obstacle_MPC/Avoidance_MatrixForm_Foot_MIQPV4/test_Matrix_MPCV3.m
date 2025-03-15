%%
clear
clc
% System Parameters
Tstep = 0.4; % step time is 0.4, fixed for now
dt = 0.1; % MPC sample Time, 
Npred = 6; % Number of steps
z0 = .9; % walking height
g = 9.81; % gravity term;
Nodes = Npred * round(Tstep / dt) + 1;

q_init = [0;0.5;0;-0.06]; % intial robot state
dx_des = 0.5;
w = sqrt(9.81 / 0.9);
x0_ref = dx_des / w * (2 - exp(w * Tstep) - exp(-w * Tstep)) / (exp(w * Tstep) - exp(-w * Tstep));
dy_des = sqrt(w) * 0.11 * tanh(sqrt(w) * Tstep/2);
dy_off = 0.0;
x_ref = dx_des * ones(Npred + 1,1);
y_ref = [0;dy_off + dy_des * (-1).^(2:Npred + 1).'];

f_length = [.8;.4];     % foot length limit
f_init = [0;-0.11];      % foot initial state
f_param = [0;0;x0_ref;0;0;0.11]; % foot parameters
Weights = [0;5000;0;5000;10000;10000;500;500;5000]; % MPC weights
r = [0.1;0.5];          % obstacle radius
qo_ic = [-0.2;0.0];      % obstacle position
qo_tan = [q_init(1);q_init(3)] - qo_ic; % obstacle tangent vector
qo_tan = qo_tan/norm(qo_tan);
du_ref = 0.9;

x_goal = 0.0 + 0.5 * 0.4;
y_goal = 0.11;
z_goal = [0.0;0.1;0.2;0.1;0.0];

swf_cq = [0;0.11;0.0];
swf_rq = [linspace(0, x_goal, 5)'; linspace(0.11, y_goal, 5)'; z_goal];
swf_Q = 100*[100;100;10];
swf_obs = [-1.35;0.1;0.0;0.1;0.15;0.0;0.25;100000;100000;0.5;0.1;0.5;100;0];

dN = round(Tstep / dt) + 1;
% Reference Foot Trajectory
swf_rx = swf_rq(1:dN);
swf_ry = swf_rq(dN + 1:2*dN);
swf_rz = swf_rq(2*dN+1:end);


swf_obs(1:3)
ref_p = [swf_rx(5);swf_ry(5);swf_rz(5)]
vec = (ref_p - swf_obs(1:3)) / norm(ref_p - swf_obs(1:3))
inter = swf_obs(1:3) + swf_obs(5) * vec
Input = [q_init;x_ref;y_ref;f_length;f_init;f_param;Weights;r;qo_ic;qo_tan;0.1;0;0;du_ref;swf_cq;swf_rq;swf_Q;swf_obs];
[a,b,c,d,e,f] = RightStart_Step0V3(Input,0*rand(234,1));

Aeq_dense = full(a);
beq_dense = -full(b);
Aiq_dense = full(c);
biq_dense = -full(d);
H_dense = full(e);
f_dense = full(f);

% Set Variable Type
var_num = size(Aeq_dense,2);
Aiq_num = size(Aiq_dense,1);
Aeq_num = size(Aeq_dense,1);

vartype = [repmat('C',var_num - 4, 1); repmat('B', 4, 1)];
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

% Set Gurobi parameters (optional)
params.outputflag = 1; % Display output

% Solve the MIQP problem
result = gurobi(model, params);
design_vector = result.x;
%%


traj_x = design_vector(1:Nodes * 2);
traj_y = design_vector(3*Nodes + Npred + 1:5*Nodes + Npred);

dPx = design_vector(3*Nodes + 1:3*Nodes + 4);
dPy = design_vector(6*Nodes + Npred + 1:6*Nodes + Npred + 4);

angle = 0:0.1:2*pi;
xoff = sin(angle);
yoff = cos(angle);

actual_foot_x = [f_init(1);f_init(1) + cumsum(dPx)];
actual_foot_y = [f_init(2);f_init(2) + cumsum(dPy)];

swing_foot = design_vector(7*Nodes + Npred + 5 : end);
swing_foot_x = swing_foot(1:5);
swing_foot_y = swing_foot(6:10);
swing_foot_z = swing_foot(11:15);

close all
figure
plot(traj_x(1:2:end),traj_y(1:2:end))
hold on
plot(qo_ic(1) + xoff *.2,qo_ic(2) + yoff *.2)
plot(qo_ic(1) + xoff *.4,qo_ic(2) + yoff *.4)

plot(actual_foot_x(1),actual_foot_y(1),'o','MarkerSize',5,    'MarkerEdgeColor','b',...
    'MarkerFaceColor','r')
plot(actual_foot_x(2),actual_foot_y(2),'o','MarkerSize',10,    'MarkerEdgeColor','b',...
    'MarkerFaceColor','g')
plot(actual_foot_x(3),actual_foot_y(3),'o','MarkerSize',15,    'MarkerEdgeColor','b',...
    'MarkerFaceColor','r')
plot(actual_foot_x(4),actual_foot_y(4),'o','MarkerSize',20,    'MarkerEdgeColor','b',...
    'MarkerFaceColor','g')
plot(swing_foot_x, swing_foot_y)
plot(swing_foot_x(1),swing_foot_y(1),'o','MarkerSize',5,    'MarkerEdgeColor','b',...
    'MarkerFaceColor','b')
xlim([-2 2])
ylim([-2 2])

[sx,sy,sz]=sphere;

figure
plot3(actual_foot_x(1),actual_foot_y(1),0,'o','MarkerSize',5,    'MarkerEdgeColor','b',...
    'MarkerFaceColor','r')
hold on
plot3(actual_foot_x(2),actual_foot_y(2),0,'o','MarkerSize',10,    'MarkerEdgeColor','b',...
    'MarkerFaceColor','g')
plot3(swing_foot_x,swing_foot_y,swing_foot_z);
hSurface = surf(swf_obs(4) * sx + swf_obs(1), swf_obs(4) * sy + swf_obs(2), swf_obs(4) * sz + swf_obs(3), 'FaceColor', 'red');
set(hSurface,'FaceColor',[0 0 1], ...
  'FaceAlpha',1,'FaceLighting','gouraud')

xlabel('x')
ylabel('y')
zlabel('z')
grid on

xlim([-0 0.6])
ylim([-.3 0.3])
zlim([0 0.6])