function H = forward(joint, myrobot)
    H = [1, 0, 0, 0; % Initializes H as identity matrix 
              0, 1, 0, 0;
              0, 0, 1, 0;
              0, 0, 0, 1];

    for i = 1:6 % iterate over each link from 1 -> 6
        theta = joint(i); % get variables associated with each link
        d = myrobot.links(i).d; % getting d variable for each joint 
        a = myrobot.links(i).a;
        alpha = myrobot.links(i).alpha;

        % calculate the H_i homogeneous tsf matrix
        A_i = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha),  a*cos(theta);
               sin(theta), cos(theta)*cos(alpha),  -cos(theta)*sin(alpha), a*sin(theta);
               0,          sin(alpha),             cos(alpha),             d;
               0,          0,                      0,                      1];

        % multiply all Ai matrices until we get our base to end effector tsf
        H = H * A_i;
    end
end