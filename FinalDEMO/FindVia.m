function [viapoint] = FindVia(final)


clearance = 75; %height above the square from which the end effector will descend vertically 
viapoint = final+ [0 0 clearance 0 0 0 0 0 0]; %add the clearance to the pos of the square
end

