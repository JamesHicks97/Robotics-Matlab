function [Dz] = FindDz(d,i)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
Dz= [1 0 0 0;
    0 1 0 0;
    0 0 1 d(i);
    0 0 0 1];
end