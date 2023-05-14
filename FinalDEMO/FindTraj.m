function [x, xdot, xdotdot, y, ydot, ydotdot, z, zdot, zdotdot] = FindTraj(coefs, t)
%Finds the position, velocity and trajectory in all three dimensions.

x = PosTraj(coefs(1,:),t);
xdot = VeloTraj(coefs(1,:),t);
xdotdot = AccTraj(coefs(1,:),t);

y = PosTraj(coefs(2,:),t);
ydot = VeloTraj(coefs(2,:),t);
ydotdot = AccTraj(coefs(2,:),t);

z = PosTraj(coefs(3,:),t);
zdot = VeloTraj(coefs(3,:),t);
zdotdot = AccTraj(coefs(3,:),t);

%combine data
Trag = [x, xdot, xdotdot, y, ydot, ydotdot, z, zdot, zdotdot];
end

