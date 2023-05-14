function [v] = VeloTraj(coefs, tvec)
%v(t) = 5*a5*t^4 + 4*a4*t^3 + 3*a3*t^2 + 2*a2*t + a1 
v = 5*coefs(6)*tvec.^4 + 4*coefs(5)*tvec.^3 + 3*coefs(4)*tvec.^2+2*coefs(3)*tvec+coefs(2);
end

