function [C, d] = InscribedEllipse(A, b, bound)
%INSCRIBEDELLIPSE Summary of this function goes here
%   Detailed explanation goes here
%   This function find the max volume ellipse 
% Problem parameters
[N, M] = size(A);

% Solve the SDP using CVX
cvx_begin sdp
    variable C(M, M) symmetric   % Symmetric positive definite matrix
    variable d(M)                % Vector d
    
    maximize log_det(C)          % Objective function: log(det(C))
    
    subject to
        C >= 0                   % C must be positive semidefinite
        for i = 1:N
            norm(A(i, :) * C) + A(i, :) * d <= b(i) - bound; % Inequality constraints
        end
cvx_end

end

