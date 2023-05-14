function [time_start,time_stop,intervals] = FindTime(initial, final, initial_time)


joint_angles1 = inverse_kinematics_f([initial(1) initial(2) initial(3)]);
joint_angles2 = inverse_kinematics_f([final(1) final(2) final(3)]);

max_diff= max(abs(joint_angles1-joint_angles2));
dps= 15.0;

time_start =initial_time;
time_stop =initial_time+(max_diff/dps);

end

