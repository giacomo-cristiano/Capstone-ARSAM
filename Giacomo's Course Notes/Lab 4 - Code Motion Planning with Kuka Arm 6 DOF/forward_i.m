function H = forward_i(num,joint, myrobot)
    H = [1, 0, 0, 0; 
              0, 1, 0, 0;
              0, 0, 1, 0;
              0, 0, 0, 1];

    for i = 1:num % iterate over each link from 1 -> 6
        theta = joint(i); % get variables associated with each link
        d = myrobot.links(i).d;
        a = myrobot.links(i).a;
        alpha = myrobot.links(i).alpha;

        % calculate the H_i homogeneous tsf matrix
        A_i = [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha),  a*cos(theta);
               sin(theta), cos(theta)*cos(alpha),  -cos(theta)*sin(alpha), a*sin(theta);
               0,          sin(alpha),             cos(alpha),             d;
               0,          0,                      0,                      1];

        % multiply matricies until we get our base to end effector tsf
        H = H * A_i;
    end
end