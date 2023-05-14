function [whole_traj] = demo_f()
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
piece_locator;% loads locations of square
load_moves;% loads moves in order

traj = MovePiece([M(1,1:3) 0 0 0 0 0 0], [M(1,4:6) 0 0 0 0 0 0], M(1,4:6),M(1,7), M(1,8)); %find position trajectory in form [x_pos; y_pos; z_pos; t]
%Note reverts back to 0 at start of each turn. Shouldn't be a problem.

    for j= 1:length(traj)
       joint_angles = inverse_kinematics_f([traj(1,j) traj(2,j) traj(3,j)]); %finds angles needed at each point in time
    end
   
whole_traj(1:5,1:length(traj)) = traj;
whole_t = traj(5,length(traj))
%repeat for the rest of the moves
for i=2:size(M,1)
    
    
    traj = MovePiece([M(i,1:3) 0 0 0 0 0 0], [M(i,4:6) 0 0 0 0 0 0], M(i-1,4:6),M(i,7), M(i,8)); 
    for j= 1:length(traj)
      joint_angles = inverse_kinematics_f([traj(1,j) traj(2,j) traj(3,j)]);
    end
    whole_traj = [whole_traj, traj];
end 
end

