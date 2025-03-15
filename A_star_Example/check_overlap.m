function [over_lap] = check_overlap(obs_pos1, obs_size1, obs_pos2, obs_size2)
%CHECK_OVERLAP Summary of this function goes here
%   Detailed explanation goes here

% Unpack state information
x1 = obs_pos1(1) - obs_size1(1);
x2 = obs_pos1(1) + obs_size1(1);
y1 = obs_pos1(2) - obs_size1(2);
y2 = obs_pos1(2) + obs_size1(2);

x3 = obs_pos2(1) - obs_size2(1);
x4 = obs_pos2(1) + obs_size2(1);
y3 = obs_pos2(2) - obs_size2(2);
y4 = obs_pos2(2) + obs_size2(2);

% Check overlap
over_lap = ~(y2<=y3 || y4<= y1 || x2<=x3 || x4<=x1);
end

