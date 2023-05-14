function [coefs] = FindCoefs(initial, final, t)
%split inputs into the respective vectors
initialpos =initial(1:3); %intial position in form [x y z]
initialvelocity = initial(4:6); %intial position in form [xdot ydot zdot]
initialacceleration = initial(7:9); %intial position in form [xdotdot ydotdot zdotdot]
finalpos = final(1:3); %final position in form [x y z]
finalvelocity = final(4:6); %final position in form [xdot ydot zdot]
finalacceleration = final(7:9); %final position in form [xdotdot ydotdot zdotdot]
initial
final
t
%Cycle through index to find all three dimensions (i=1 finds x dimension,
%i=2 find y and i=3 find z)
for i=1:3
    %Use matrix multiplication of equation laid out in report to find
    %coefficients
    A = [1 t(1) t(1)^2 t(1)^3 t(1)^4 t(1)^5; 1 t(2) t(2)^2 t(2)^3 t(2)^4 t(2)^5; 0 1 2*t(1) 3*t(1)^2 4*t(1)^3 5*t(1)^4; 0 1 2*t(2) 3*t(2)^2 4*t(2)^3 5*t(2)^4; 0 0 2 6*t(1) 12*t(1)^2 20*t(1)^3; 0 0 2 6*t(2) 12*t(2)^2 20*t(2)^3];
    B = [initialpos(i); finalpos(i); initialvelocity(i); finalvelocity(i); initialacceleration(i); finalacceleration(i)];

    coefs(i,:) = inv(A)*B;

end

