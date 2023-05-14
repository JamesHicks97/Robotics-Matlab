function [grip] = FindGrip(piece)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
if piece== 1;
    grip=0.05;
elseif piece==2
    grip=0.10;
elseif piece==3
    grip=0.15;
elseif piece==4
    grip=0.20;
elseif piece==5
    grip=0.25;
elseif piece==6
    grip=0.30;
end

