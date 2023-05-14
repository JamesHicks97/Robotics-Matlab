function [Rz] = FindRz(theta,i)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
Rz= [cos(theta(i)) -sin(theta(i)) 0 0;
    sin(theta(i)) cos(theta(i)) 0 0;
    0 0 1 0;
    0 0 0 1];
end