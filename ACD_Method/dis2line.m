function [dis] = dis2line(xint,xend,point)
%DIS2LINE Summary of this function goes here
%   Detailed explanation goes here

vec1 = xend - xint;
vec2 = point - xint;

proj = (vec1' * vec2) / (vec1' * vec1) * vec1;
orth = vec2 - proj;
dis = norm(orth,2);
end

