function [angles_out] = fit2robot(angles_in)
%FIT2ROBOT function to fit the angles provided by the inverse kinematics
%function to the pyhsical robot of AG17. This is necessary because of
%offset unpreventable in the physical robot

% Defining the offsets (measured by positioning the robot so that
% all angles should be zero, then reading the angle feedback)
offsets = deg2rad([-6 7 -4 0]);

% Defining the angles to output
angles_out = angles_in;

% Joint 3 = Q2+Q3 since the motor for Q3 is mounted to the baseplate
angles_out(3) = angles_out(2)  + angles_out(3);

% Reversing the value for motor 2 since it moves inverted
angles_out(2) = -angles_out(2);

% Adding the offsets
angles_out = angles_out + offsets;

end

