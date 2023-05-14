function [traj] = MovePiece(start, finish, lastmove, index, piece)
%Finds the trajectories of moving a piece from start location

home_pos  = [100 0 170]; %Position vector for first point in form [x_pos y_pos z_pos] 
home_v    = [0 0 0];    %Velocity vector for first point in form [x_pos y_pos z_pos]
home_a    = [0 0 0];    %Acceleration vector for first point in form [x_pos y_pos z_pos]
home = [home_pos home_v home_a]; %combine into single vector

begin = FindVia([lastmove 0 0 0 0 0 0]);


intervals = 60;%how many timepoints each segement
grip_time = 0.5;
%define viapoint 1
viapoint1 = FindVia(start); %Position vector for first viapoint in form [x_pos y_pos z_pos]
%define viapoint 2
viapoint2 = FindVia(finish); %Position vector for first viapoint in form [x_pos y_pos z_pos]


open_grip = 0.22;%%%Insert open end effector angle in radians
closed_grip = FindGrip(piece);


%if the second move of a two part turn start from the viapoint above the
%last square moved to
if index == 2
    
    [x,y]=FindTime(begin, viapoint1,0);
    t_a(1)=x;
    t_a(2)=y;
    coefs_A = FindCoefs(begin, viapoint1, t_a);
    tvec_a = linspace(t_a(1),t_a(2),intervals);
    grip_a = open_grip*ones(1,length(tvec_a));
    
    [x_a, xdot_a, xdotdot_a, y_a, ydot_a, ydotdot_a, z_a, zdot_a, zdotdot_a] = FindTraj(coefs_A, tvec_a);
else %else start from home
    [x,y]=FindTime(home, viapoint1,0);
    t_a(1)=x;
    t_a(2)=y;
    coefs_A = FindCoefs(home, viapoint1, t_a);
    tvec_a = linspace(t_a(1),t_a(2),intervals);
    grip_a = open_grip*ones(1,length(tvec_a));

    [x_a, xdot_a, xdotdot_a, y_a, ydot_a, ydotdot_a, z_a, zdot_a, zdotdot_a] = FindTraj(coefs_A, tvec_a);
end



%solve the rest of the polynomials
[x,y]=FindTime(viapoint1, start, t_a(2));
t_b(1)=x;
t_b(2)=y;
coefs_B = FindCoefs(viapoint1, start, t_b);
tvec_b = linspace(t_b(1),t_b(2),intervals);
grip_b = open_grip*ones(1,length(tvec_b));

[x_b, xdot_b, xdotdot_b, y_b, ydot_b, ydotdot_b, z_b, zdot_b, zdotdot_b] = FindTraj(coefs_B, tvec_b);


t_close(1)=t_b(2);
t_close(2)=t_b(2)+grip_time;
coefs_close = FindCoefs(start, start, t_close);
tvec_close = linspace(t_close(1),t_close(2),20);
grip_close = GripChange(open_grip, closed_grip,tvec_close );

[x_close, xdot_close, xdotdot_close, y_close, ydot_close, ydotdot_close, z_close, zdot_close, zdotdot_close] = FindTraj(coefs_close, tvec_close);

[x,y]=FindTime(start, viapoint1, t_close(2));
t_c(1)=x;
t_c(2)=y;
coefs_C = FindCoefs(start, viapoint1, t_c);
tvec_c = linspace(t_c(1),t_c(2),intervals);
grip_c = closed_grip*ones(1,length(tvec_c));

[x_c, xdot_c, xdotdot_c, y_c, ydot_c, ydotdot_c, z_c, zdot_c, zdotdot_c] = FindTraj(coefs_C, tvec_c);


[x,y]=FindTime(viapoint1, viapoint2, t_c(2));
t_d(1)=x;
t_d(2)=y;
coefs_D = FindCoefs(viapoint1, viapoint2, t_d);
tvec_d = linspace(t_d(1),t_d(2),intervals);
grip_d = closed_grip*ones(1,length(tvec_d));
[x_d, xdot_d, xdotdot_d, y_d, ydot_d, ydotdot_d, z_d, zdot_d, zdotdot_d] = FindTraj(coefs_D, tvec_d);

[x,y]=FindTime(viapoint2, finish, t_d(2));
t_e(1)=x;
t_e(2)=y;
coefs_E = FindCoefs(viapoint2, finish, t_e);
tvec_e = linspace(t_e(1),t_e(2),intervals);
grip_e = closed_grip*ones(1,length(tvec_d));
[x_e, xdot_e, xdotdot_e, y_e, ydot_e, ydotdot_e, z_e, zdot_e, zdotdot_e] = FindTraj(coefs_E, tvec_e);


t_open(1)=t_e(2);
t_open(2)=t_e(2)+grip_time;
coefs_open = FindCoefs(finish, finish, t_open);
tvec_open = linspace(t_open(1),t_open(2),20);
grip_open = GripChange(closed_grip, open_grip, tvec_open);
[x_open, xdot_open, xdotdot_open, y_open, ydot_open, ydotdot_open, z_open, zdot_open, zdotdot_open] = FindTraj(coefs_open, tvec_open);

[x,y]=FindTime(finish, viapoint2, t_open(2));
t_f(1)=x;
t_f(2)=y;
coefs_F = FindCoefs(finish, viapoint2, t_f);
tvec_f = linspace(t_f(1),t_f(2),intervals);
grip_f = open_grip*ones(1,length(tvec_f));
[x_f, xdot_f, xdotdot_f, y_f, ydot_f, ydotdot_f, z_f, zdot_f, zdotdot_f] = FindTraj(coefs_F, tvec_f);

if index== 1
    tvec = [tvec_a tvec_b tvec_close tvec_c tvec_d tvec_e tvec_open tvec_f];
    x = [x_a x_b x_close x_c x_d x_e x_open x_f];
    y = [y_a y_b y_close y_c y_d y_e y_open y_f];
    z = [z_a z_b z_close z_c z_d z_e z_open z_f];
    grip = [grip_a grip_b grip_close grip_c grip_d grip_e grip_open grip_f];
    traj = [x;y;z;grip;tvec];
else
    [x,y]=FindTime(viapoint2, home, t_f(2));
    t_g(1)=x;
    t_g(2)=y;
    coefs_G = FindCoefs(viapoint2, home, t_g);
    tvec_g = linspace(t_g(1),t_g(2),intervals);
    grip_g = open_grip*ones(1,length(tvec_g));
    [x_g, xdot_g, xdotdot_g, y_g, ydot_g, ydotdot_g, z_g, zdot_g, zdotdot_g] = FindTraj(coefs_G, tvec_g);
    %combine polynomials
    tvec = [tvec_a tvec_b tvec_close tvec_c tvec_d tvec_e tvec_open tvec_f tvec_g];
    x = [x_a x_b x_close x_c x_d x_e x_open x_f x_g];
    y = [y_a y_b y_close y_c y_d y_e y_open y_f y_g];
    z = [z_a z_b z_close z_c z_d z_e z_open z_f z_g];
    grip = [grip_a grip_b grip_close grip_c grip_d grip_e grip_open grip_f grip_g];
    traj = [x;y;z;grip; tvec];
end

end

