%% Assignment two group 17

clear
clc
syms Q1 Q2 Q3 Q4 l1 l2 l3 l4 Q1_dot Q2_dot Q3_dot Q4_dot; % define angles dues to acctuation as symbolic variables

%\\\\\\ Part 2.1
% symbolic variables given values 

l1= 70 / 1000; % length of first member in m
l2= 200 / 1000; % length of second member in m
l3= 240 / 1000; % length of third member in m
l4=100 / 1000; % length of fourth member in m

% These are commented out for final submition but were used when checking
% values during the project
% Q1 = sym(pi)/4; %angular position of joint 1 relative to zero postion in radians
% Q2 = sym(pi)/4; %angular position of joint 2 relative to zero postion in radians
% Q3 = -sym(pi)/4; %angular position of joint 3 relative to zero postion in radians
% Q4 = 0 %angular position of joint 4 relative to zero postion in radians
% 
% Q1_dot = 1; %angular velocity of frame 1 relative to frame 0
% Q2_dot = 2; %angular velocity of frame 2 relative to frame 1
% Q3_dot = -1; %angular velocity of frame 3 relative to frame 2
% Q4_dot = 0; %angular velocity of frame E relative to frame 3


%dh table  for 'hook' configuration

a= [0 0 l2 l3 0];
alpha = [0 -sym(pi)/2 0 0 -sym(pi)/2];
d = [l1 0 0 0 l4];
theta = [Q1 -sym(pi)/2+Q2 sym(pi)/2+Q3 -Q2-Q3 0];

% Transformation Matrices
T1 = FindT(a, alpha, d, theta, 1);
T2 = FindT(a, alpha, d, theta, 2);
T3 = FindT(a, alpha, d, theta, 3);
T4 = FindT(a, alpha, d, theta, 4);
T5 = FindT(a, alpha, d, theta, 5);

T = T1*T2*T3*T4*T5; %find transform matrix from frame 0 to end frame

%\\\\ Part 2.2
% Rotation matrices

r1 = T1(1:3,1:3);
r2 = T2(1:3,1:3);
r3 = T3(1:3,1:3);
r4 = T4(1:3,1:3);
r5 = T5(1:3,1:3);


% find zhat of each joint in the zero frame

zhat_0 = [0;0;1]; 

zhat_1 = r1*zhat_0; %direction of z1 in zero frame
zhat_2 = r1*r2*zhat_0; %direction of z2 in zero frame
zhat_3 = r1*r2*r3*zhat_0 ;%direction of z3 in zero frame
zhat_4 = r1*r2*r3*r4*zhat_0;%direction of z4 in zero frame
zhat_E = simplify(r1*r2*r3*r4*r5*zhat_0);%direction of z5 in zero frame

 O = [0 ;0 ;0 ;1]; %origin point vector

 %Find the distance from origin to each point in zero frame
 P1 = T1*O; % distance from Origin to frame 1
 P2 = T1*T2*O; % distance from Origin to frame 2
 P3 = T1*T2*T3*O; % distance from Origin to frame 3
 P4 = T1*T2*T3*T4*O; % distance from Origin to frame 4
 PE = T1*T2*T3*T4*T5*O; % distance from Origin to frame E
 
 %use vector subtraction to find distance from each joint to end effector
 P1E = PE-P1; % distance frame 1 to frame E
 P2E = PE-P2; % distance frame 2 to frame E
 P3E = PE-P3; % distance frame 3 to frame E
 P4E = PE-P4; % distance frame 4 to frame E
 
 P1E = simplify(P1E(1:3));
 P2E = simplify(P2E(1:3));
 P3E = simplify(P3E(1:3));
 P4E = simplify(P4E(1:3));
 
 
% Find Velocity Jacobian from lecture 4 slide 9

Jv1 = cross(zhat_1, P1E);
Jv2 = cross(zhat_2, P2E);
Jv3 = cross(zhat_3, P3E);
Jv4 = cross(zhat_4, P4E);

Jv= [Jv1 Jv2 Jv3 Jv4];

% Find Angular jacobian from lecture 4 slide 10

Jw = [zhat_1 zhat_2 zhat_3 zhat_4];
% Find Jacobian
disp('The Jacobian with symbolic joint variables is:')
J=simplify([Jv;Jw]) %combine jacobian elements
Q_dot = [Q1_dot; Q2_dot; Q3_dot; Q4_dot];

%\\\\\\ Part 2.3
disp('The translational and angular velocities with symbolic joint variables via Jacobian method are:')
VandOmega = J*Q_dot %find the Translational and angular components of velocity by multiplying jacobian with angular velocites at each joint.
disp('The translational and angular velocities with actual joint variables via Jacobian method are:')
VandOmega_final = subs(VandOmega, {Q1, Q2, Q3, Q4, Q1_dot, Q2_dot, Q3_dot, Q4_dot}, {0, 0, 0, 0, 1, 0,0,0}) %sub in values for each joints angular position and velocity

% Find translation velocity secondary dynamics method
disp('The translational velocities with actual joint variables via secondary dynamics method for system [0 0 0 0 1 0 0 0] are:')
V = cross(Q1_dot*zhat_1,P1E)+ cross(Q2_dot*zhat_2,P2E)+cross(Q3_dot*zhat_3,P3E)+cross(Q4_dot*zhat_4,P4E);
V_Method2 = subs(V, {Q1, Q2, Q3, Q4, Q1_dot, Q2_dot, Q3_dot, Q4_dot}, {0, 0, 0, 0, 1, 0, 0, 0})% calculate end effectors translational velocity using second method
omega = Q1_dot*zhat_1+Q2_dot*zhat_2+Q3_dot*zhat_3+Q4_dot*zhat_4; % calculate end effectors translational velocity using second method
disp('The angular velocities with actual joint variables via secondary dynamics method for system [0 0 0 0 1 0 0 0] are:are:')
omega_Method2 = subs(omega, {Q1, Q2, Q3, Q4, Q1_dot, Q2_dot, Q3_dot, Q4_dot}, {0, 0, 0, 0, 1, 0, 0, 0})

disp('The translational velocities with actual joint variables via secondary dynamics method for system [45 45 -45 0 1 2 -1 0] are:')
V = cross(Q1_dot*zhat_1,P1E)+ cross(Q2_dot*zhat_2,P2E)+cross(Q3_dot*zhat_3,P3E)+cross(Q4_dot*zhat_4,P4E);
V_Method2 = subs(V, {Q1, Q2, Q3, Q4, Q1_dot, Q2_dot, Q3_dot, Q4_dot}, {deg2rad(45) deg2rad(45) deg2rad(-45) 0 1 2 -1 0})% calculate end effectors translational velocity using second method
omega = Q1_dot*zhat_1+Q2_dot*zhat_2+Q3_dot*zhat_3+Q4_dot*zhat_4; % calculate end effectors translational velocity using second method
disp('The angular velocities with actual joint variables via secondary dynamics method for system [45 45 -45 0 1 2 -1 0] are:are:')
omega_Method2 = subs(omega, {Q1, Q2, Q3, Q4, Q1_dot, Q2_dot, Q3_dot, Q4_dot}, {deg2rad(45) deg2rad(45) deg2rad(-45) 0 1 2 -1 0})


%\\\\\\\ Part 3.1
% Deriving the Jacobian matrices for the centres of mass

% symbolic values for the centers of mass measured from the start of each
% link
syms lc1 lc2 lc3 lc4


% Defining the vectors of the Centres of Mass in each frame
pc1 = [0; 0; lc1];
pc2 = [lc2; 0; 0];
pc3 = [lc3; 0; 0];
pc4 = [0; lc4; 0];

% Transforming the vectors back to the 0 frame
% pc1 = r1*pc1; pc1 is not being transformed as it is expressed in frame 0
% already
pc2 = r1*r2*pc2;
pc3 = r1*r2*r3*pc3;
pc4 = r1*r2*r3*r4*pc4;

% Defining the translational velocity part of the Jacobians
null_vector = [0; 0; 0];

Jvc1 = [cross(zhat_1, pc1) null_vector null_vector null_vector];
Jvc2 = [cross(zhat_1, pc2) cross(zhat_2, pc2) null_vector null_vector];
Jvc3 = [cross(zhat_1, pc3) cross(zhat_2, pc3) cross(zhat_3, pc3) null_vector];
Jvc4 = [cross(zhat_1, pc4) cross(zhat_2, pc4) cross(zhat_3, pc4) cross(zhat_4, pc4)];

% Defining the angular velocity part of the Jacobians
Jwc1 = [zhat_1 null_vector null_vector null_vector];
Jwc2 = [zhat_1 zhat_2 null_vector null_vector];
Jwc3 = [zhat_1 zhat_2 zhat_3 null_vector];
Jwc4 = [zhat_1 zhat_2 zhat_3 zhat_4];

% Defining the final Jacobians by combining Jvci and Jwci
disp('The final Jacobians for the joints are the  following:')
Jc1 = simplify([Jvc1; Jwc1])
Jc2 = simplify([Jvc2; Jwc2])
Jc3 = simplify([Jvc3; Jwc3])
Jc4 = simplify([Jvc4; Jwc4])


%\\\\\\ Part 3.2
% Calculating the torque for each joint
% Defining the gravity forces
g = 9.81;
fg_1 = -0.039332 * g; % Replace values with actual masses
fg_2 = -0.062712 * g;
fg_3 = -0.075322 * g;
fg_4 = -0.026991 * g;


Fg_1 = [0; 0; fg_1; 0; 0; 0];
Fg_2 = [0; 0; fg_2; 0; 0; 0];
Fg_3 = [0; 0; fg_3; 0; 0; 0];
Fg_4 = [0; 0; fg_4; 0; 0; 0];


% Calculating the torque necessary to support the robot's own weight
tau_G = Jc1' * Fg_1 + Jc2' * Fg_2 + Jc3' * Fg_3 + Jc4' * Fg_4;


% Defining the gravity force on the end effector exerted by the heaviest
% chess piece
f_cp = -0.008 * g;

F_ext = [0; 0; f_cp; 0; 0; 0]; % no moments exerted on end-effector

% Calculating the torque necessary to counteract the force exerted by the
% chesspiece
tau_ext = J' * F_ext;

% Total torque is the sum of both calculated torques
tau = tau_ext + tau_G;


% Iterating through all positions in the workspace to find the maximum torques
% Defining the joint limits
joint_limits = [-90, 90, -45, 135, 0, 150, -145, 145];


link_c1 = 40.259 / 1000; % length to first centre of mass in m!
link_c2 = 95.74 / 1000; % length to second centre of mass in m
link_c3 = 73.72 / 1000; % length to third centre of mass in m
link_c4 = 57.958 / 1000; % length to fourth centre of mass in m

% Substituting the actual distances of the centres of mass for their
% symbolic values
tau = subs(tau, {lc1, lc2, lc3, lc4}, {link_c1, link_c2, link_c3, link_c4});

% Initialising a vector with the initial maximum torques on each joint
tau_max = [0; 0; 0; 0];

% Stepwidth for looking throught he angles in degrees
% (width increased for 10 here for faster running code; values in report
% obtained with a stepwidth of 5 degrees)

stepwidth = 10;

for q1 = deg2rad(joint_limits(1)):deg2rad(stepwidth):deg2rad(joint_limits(2))% rotate the 1st joint
    for q2 = deg2rad(joint_limits(3)):deg2rad(stepwidth):deg2rad(joint_limits(4)) % rotate the 2nd joint from -45 degrees from its intial pos to +135 degrees
        for q3 = deg2rad(joint_limits(5)):deg2rad(stepwidth):deg2rad(joint_limits(6)) % rotate the 3rd joint from 0 degrees from its intial pos to +150 degrees
            % Testing if the joints limits for q4 are kept
            if joint_limits(7) < -(q2+q3) && -(q2+q3) < joint_limits(8)
                
                %substituting in the values for each current angle
                tau_new = subs(tau, {Q1, Q2, Q3, Q4}, {q1, q2, q3, -(q2+q3)});

                % substituting the new value for a joint if the found torque is
                % higher than the previous maximum
                for i  = 1:length(tau_max)
                    if tau_new(i) > tau_max(i)
                        tau_max(i) = tau_new(i);
                    end
                end
            end

        end 

    end
end

disp('The maximum torques on joints 1-4 are:')
tau_max
