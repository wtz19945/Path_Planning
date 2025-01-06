% Add MOSEK path (adjust based on your MOSEK installation)

% Problem parameters
N = 5;                  % Number of constraints
M = 3;                  % Dimension of matrix C and vector d
A = randn(N, M);        % Random constraint matrix (rows are a_i^T)
b = rand(N, 1) + 1;     % Positive bounds for feasibility

% Initialize MOSEK problem structure
[~, res] = mosekopt('symbcon echo(0)');  % Get MOSEK symbolic constants
symbcon = res.symbcon;

% Define problem structure
prob = struct();
prob.c = [];                  % No explicit linear objective
prob.a = [];                  % Start with no explicit linear constraints
prob.blc = [];                % Lower bounds on constraints
prob.buc = [];                % Upper bounds on constraints

% Semidefinite variable: C (M x M symmetric positive semidefinite matrix)
prob.bardim = M;              % Dimension of semidefinite variable C
prob.barc.subj = [];          % SDP cone indices
prob.barc.subk = [];          % SDP row indices
prob.barc.subl = [];          % SDP column indices
prob.barc.val = [];           % SDP values

% Add objective: Maximize log(det(C))
% MOSEK handles this internally via the SDP primal-dual solver.

% Initialize constraint setup
cone_offset = 0;

% Add the constraints ||a_i^T C|| + a_i^T d <= b_i for each i
for i = 1:N
    ai = A(i, :)';            % Extract a_i as column vector
    
    % ||a_i^T C|| is modeled using MOSEK's quadratic cone
    prob.a = [prob.a; sparse(1, M)];       % Add space for linear part
    prob.blc = [prob.blc; -inf];           % No lower bound
    prob.buc = [prob.buc; b(i)];           % Upper bound b_i

    % Handle SDP terms
    for j = 1:M
        for k = j:M
            prob.barc.subj = [prob.barc.subj, i + cone_offset];
            prob.barc.subk = [prob.barc.subk, j];
            prob.barc.subl = [prob.barc.subl, k];
            prob.barc.val = [prob.barc.val, ai(j) * ai(k)];
        end
    end
end

% Solve the problem using MOSEK
[~, res] = mosekopt('minimize echo(0)', prob);

% Extract solution
C_opt = reshape(res.sol.itr.barx, [M, M]);  % Reshape C from solution
d_opt = res.sol.itr.xx(1:M);                % Extract vector d

% Display results
disp('Optimal matrix C:');
disp(C_opt);

disp('Optimal vector d:');
disp(d_opt);
