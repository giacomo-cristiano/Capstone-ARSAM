function H = forward_link(joint, myrobot, link_num)
    % Computes the forward kinematics up to a specific link.
    % Inputs:
    %   joint - Vector of joint angles [1x6] or [6x1]
    %   myrobot - The SerialLink robot model
    %   link_num - The link number for which FK is computed
    % Output:
    %   H - 4x4 homogeneous transformation matrix for the specified link

    % Initialize transformation matrix as identity
    H = eye(4);

    % Compute FK by multiplying transformation matrices from joint 1 to link_num
    for i = 1:link_num  
        theta = joint(i); 
        d = myrobot.links(i).d; 
        a = myrobot.links(i).a;
        alpha = myrobot.links(i).alpha;

        % Compute transformation for link i
        A_i = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha),  a*cos(theta);
               sin(theta), cos(theta)*cos(alpha),  -cos(theta)*sin(alpha), a*sin(theta);
               0,          sin(alpha),             cos(alpha),             d;
               0,          0,                      0,                      1];

        % Multiply transformations
        H = H * A_i;
    end
end
