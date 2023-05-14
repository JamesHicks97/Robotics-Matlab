function [goal_traj] = slow_zero(curr_angles, goal_angles)
%SLOW_ZERO function to create the trajectory to slowly drive the robot from
% the current position to a desired one (given in angles)


% Code in case goal position is provided, not angles
% (not needed atm)
%goal_angles = deg2rad(inverse_kinematics_f(goal_pos));
%goal_angles = fit2robot(goal_angles);

% Calculating the maximum difference in angle
maxdiff = max(abs(goal_angles(1:4) - curr_angles));



% Calulcating the number of steps in the trajectory from the  maxdiff
num_steps = round((30/deg2rad(10))*maxdiff);

% Initializing the goal trajectory

goal_traj = zeros(size(curr_angles,2), num_steps);

% Building the steps for each joint
for idx = 1:size(curr_angles, 2)
    goal_traj(idx,:) = linspace(curr_angles(idx), goal_angles(idx), num_steps);
end


end

