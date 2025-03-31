function q = inverse_kuka(H, myrobot)
    % hard coded values of our problem space
    d1 = 400;
    a1 = 25;
    a2 = 315;
    a3 = 35;
    d4 = 365;
    a6 = 296.23;
    d6 = 161.44;

    % Compute wrist center coords; setup for IPK
    R6_0 = H(1:3,1:3);
    o6_0 = H(1:3,4);
    oc_0 = o6_0 - (R6_0 * [-a6;0;d6]);
    x_c = oc_0(1);
    y_c = oc_0(2);
    z_c = oc_0(3);

    % Inverse Position Kinmeatics
    % all formulas derived from prelab
    % Theta1
    q(1) = atan2(y_c, x_c); % - atan2(-d2, real(sqrt(x_c^2 + y_c^2 - d2^2)));

    % Theta3
    % D = (a2^2 + d4^2 - (z_c - d1)^2 - (x_c^2 + y_c^2 - d2^2))/(2*a2*d4);
    r = sqrt(x_c^2 + y_c^2) - a1;
    s = z_c - d1;
    D = (r^2 + s^2 - (a2^2 + a3^2 + d4^2))/(2 * a2 * sqrt(a3^2 + d4^2));
    q(3) = atan2(D, real(sqrt(1 - D^2))) - atan2(a3,d4); % elbow up solution
    
    % Theta2
    f = sqrt(a3^2 + d4^2);
    q(2) = atan2(s, r) - atan2(-f * cos(q(3) + atan2(a3,d4)), a2 + f*sin(q(3) + atan2(a3,d4)));

    % compute R6_3; setup for IOK
    H3_0 = [1, 0, 0, 0; 
              0, 1, 0, 0;
              0, 0, 1, 0;
              0, 0, 0, 1];

    for i = 1:3 % iterate over each link from 1 -> 3
        theta = q(i); % get variables associated with each link
        d = myrobot.links(i).d;
        a = myrobot.links(i).a;
        alpha = myrobot.links(i).alpha;

        % calculate the H_i homogeneous tsf matrix
        A_i = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha),  a*cos(theta);
               sin(theta), cos(theta)*cos(alpha),  -cos(theta)*sin(alpha), a*sin(theta);
               0,          sin(alpha),             cos(alpha),             d;
               0,          0,                      0,                      1];

        % multiply matricies until we get our base to end effector tsf
        H3_0 = H3_0 * A_i;
    end
    R3_0 = H3_0(1:3,1:3);
    R6_3 = R3_0.' * R6_0;

    % Inverse Orientation Kinematics (elbow up solution)
    q(4) = atan2(R6_3(2,3), R6_3(1,3));
    q(5) = atan2(real(sqrt(1 - R6_3(3,3)^2)), R6_3(3,3));
    q(6) = atan2(R6_3(3,2), -R6_3(3,1));
    % or
    % q(4) = atan2(-R6_3(2,3), -R6_3(1,3));
    % q(5) = atan2(-real(sqrt(1 - R6_3(3,3)^2)), R6_3(3,3));
    % q(6) = atan2(-R6_3(3,2), R6_3(3,1));


end