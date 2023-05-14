function [Rx] = FindRx(alpha,i)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
Rx= [1 0 0 0; 
    0 cos(alpha(i)) -sin(alpha(i)) 0;
    0 sin(alpha(i)) cos(alpha(i)) 0;
    0 0 0 1];
end

