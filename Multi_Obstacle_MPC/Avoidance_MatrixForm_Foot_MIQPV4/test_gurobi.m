% Define the problem data
Q = [1 0; 0 1]; % Quadratic coefficient matrix
c = [2; 3]; % Linear coefficient vector
A = [1 2; 4 5; 3 1]; % Constraint matrix
b = [20; 45; 27]; % Right-hand side vector

% Variable types ('C' for continuous, 'I' for integer)
vartype = ['C'; 'B'];

% Define lower and upper bounds for variables
lb = [0; 0];
ub = [Inf; Inf];

% Create a Gurobi model structure
model.Q = sparse(Q);
model.obj = c;
model.A = sparse(A);
model.rhs = b;
model.sense = '<';
model.vtype = vartype;
model.lb = lb;
model.ub = ub;

% Set Gurobi parameters (optional)
params.outputflag = 1; % Display output

% Solve the MIQP problem
result = gurobi(model, params);

% Display the results
if strcmp(result.status, 'OPTIMAL')
    fprintf('Optimal objective: %f\n', result.objval);
    fprintf('Optimal solution:\n');
    disp(result.x);
else
    fprintf('No optimal solution found. Status: %s\n', result.status);
end

%%
clc
clear
names = {'x'; 'y'; 'b1'; 'b2'; 'b3'; 'b4'};

M = 1000;
A = [0 0 1 1 1 1;
    1 0 M 0 0 0;
    0 1 M 0 0 0;
    -1 0 0 M 0 0;
    0 -1 0 M 0 0;
    -1 0 0 0 M 0;
    0 1 0 0 M 0;
    1 0 0 0 0 M;
    0 -1 0 0 0 M;
    1 1 0 0 0 0;];
model.A = sparse(A);
model.obj = [1 1 0 0 0 0].';
model.rhs = [1; M;M;M;M;M;M;M;M;1];
model.sense = '=<<<<<<<<>';
model.vtype = ['C';'C';'B';'B';'B';'B'];
model.modelsense = 'min';
model.varnames = names;
model.start = [2 ;2; 0; 1; 0; 0];
model.lb = [-100;-100;-100;-100;-100;-100];
model.ub = [100;100;100;100;100;100];
params.outputflag = 1;

result = gurobi(model, params);

disp(result);

for v=1:length(names)
    fprintf('%s %d\n', names{v}, result.x(v));
end

fprintf('Obj: %e\n', result.objval);
