function [Dx] = FindDx(a,i)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
Dx= [1 0 0 a(i);
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
end

