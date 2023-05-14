function Q = inverse_kinematics_f(r)
%This function calculates the inverse kinematics for a given position in
%space.
%INPUTS:    r: the vector with positions in x, y and z for which to find IK
%           l1, l2, l3, l4: the link lengths of the robot
%           joint_limits: the limits for the joints in degrees
%OUTPUTS:  Q: the vector with the angles for q1, q2, q3, q4 in degrees


l1= 79.5;
l2=200;
l3=240;
l4=170;


joint_limits = [-90, 90, -60, 90, -50, 170, -180, 180];

% Catching the cases where the first calculation for q1 would divide by 0
    if r(1) == 0
        if r(2) > 0
            q1 = pi/2;
        elseif r(2) < 0
            q1 = -pi/2;
        else
            q1 = 0;
        end
    else
        % Calculating the angle of q1 according to the documentation
        q1 = atan(r(2)/r(1));
    end

    % Defining r_w as the vector for the point W in frame 1
    % r_w = [(r(1)/cos(q1)) 0 (r(3)+l4)];

    r_w = [(r(1) * cos(q1) + r(2) * sin(q1)), (-r(1) * sin(q1) + r(2) * cos(q1)), (r(3)+l4)];
    % r_w = [r(1) r(2) r(3)+l4]
    
    % Calculating the length of L

    L = sqrt(r_w(1)^2 + (r_w(3) - l1)^2);

    if L == 0
        error('These coordinates cannot be reached without collision (L = 0)')
    elseif L >= (l2 + l3)
        error('These coordinates are outside of the reachable workspace of the robot (L too large)')
    end

    
    % Catching the case when a divison by 0 would occur, and then calculating
    % q3 according to the documentation
    if l2 == 0 || l3 == 0
        error('The robot''s link lengths are not plausible (l2 or l3 is 0)')
    else
        q3 = acos((L^2 - l2^2 - l3^2)/(2 * l2 * l3));
    end

    % Calculating gamma first for clearer code structure
    gamma = pi - q3;

    beta = asin((l3*sin(gamma))/L);
    
    
    alpha = atan(r_w(1)/((r_w(3) - l1)));


    q2 = alpha - beta;
    
    Q = rad2deg([q1 q2 -pi/2+q3 -(q2+q3)]);
    if Q == real(Q)
        % do nothing as the Q values are valid
    else
        error('These coordinates cannot be reached (complex numbers for angles)')
        % in case acos for example is called outside the range -1;1
    end
    
    % Testing if joint angles are inside the defined limits
    if joint_limits(1) <= Q(1) && Q(1) <= joint_limits(2) && joint_limits(3) <= Q(2) && Q(2) <= joint_limits(4) && joint_limits(5) <= Q(3) && Q(3) <= joint_limits(6) && joint_limits(7) <= Q(4) && Q(4) <= joint_limits(8)
        % do nothing as the Q values are valid
    else
        error('Joint angles would be outside the defined limits')
    end
        
end

