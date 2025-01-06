map = [
    0 0 0 1 0 0;
    0 1 0 1 0 0;
    0 1 0 0 0 0;
    0 1 0 1 0 0;
    0 1 0 0 0 0;
    0 1 1 1 0 0
];


start = [5,1];
goal = [1,6];

heuristic = @(start,goal) norm(start-goal);

% Define the range of i and j
i = -1:1;
j = -1:1;

% Generate all combinations using ndgrid
[gridI, gridJ] = ndgrid(i, j);

% 8 way connected path
neighbor = [gridI(:), gridJ(:)];
% % 4 way connected path
% neighbor = [-1 0;1 0;0 1;0 -1];
path = flip(a_star(start,goal,map, heuristic, neighbor));

%
visualize_map(map, path)

