function [a] = AccTraj(coefs, tvec)
%a(t) = 20*a5*t^3 + 12*a4*t^2 + 6*a3*t + 2*a2  
a = 20*coefs(6)*tvec.^3 + 12*coefs(5)*tvec.^2 + 6*coefs(4)*tvec+2*coefs(3);
end