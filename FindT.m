function [T] = FindT(a, alpha, d, theta, i)

Rx = FindRx(alpha,i);
Dx = FindDx(a, i);
Rz = FindRz(theta, i);
Dz = FindDz(d, i);

A = Rx * Dx;
B = Rz * Dz;
T = A * B;
end

