function [pos] = PosTraj(coefs, tvec)

%pos(t) = a5*t^5 + a4*t^4 + a3*t^3 + a2*t^2 + a1*t + a0 
pos = coefs(6)*tvec.^5 + coefs(5)*tvec.^4 + coefs(4)*tvec.^3+coefs(3)*tvec.^2+coefs(2)*tvec+coefs(1);
end

