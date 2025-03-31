function Lab1()
    %% 4.1 Setup
    % all units converted to meters
    a2 = real(sqrt(43.18^2 + 2.03^2));
    d1 = 76;
    d2 = -(38.65 - 15);
    d4 = 43.18;
    d6 = 20;

    % DH table (theta, d, a, alpha)
    DH = [0, d1, 0,  pi/2;
          0, d2, a2, 0;
          0, 0,  0,  pi/2;
          0, d4, 0,  -pi/2;
          0, 0,  0,  pi/2;
          0, d6, 0,  0];

    myrobot = mypuma560(DH); % intialize robot

    %% 4.2 Plot sample joint space trajectory
    theta1 = linspace(0,pi,200);
    theta2 = linspace(0,pi/2,200);
    theta3 = linspace(0,pi,200);
    theta4 = linspace(pi/4,3*pi/4,200);
    theta5 = linspace(-pi/3,pi/3,200);
    theta6 = linspace(0,2*pi,200);
    q = [theta1;theta2;theta3;theta4;theta5;theta6].'; % 200 x 6 matrix

    plot(myrobot, q)

    %% 4.3 Forward Kinematics
    o = zeros(200,3); % initialize 200x3 matrix of zeros
    for i = 1:200
        full_H = forward(q(i,:), myrobot); % calls forward function to compute 6 joint values in each row of q at each time step based on our DH table
        o(i,:) = full_H(1:3,4); % just getting us the coordinate column from each H time step (1:3 is first 3 rows and 4 is the column)
    end
    plot3(o(:,1),o(:,2),o(:,3),'r'); % x y z coordinate plotting from 
    hold on
    plot(myrobot,q);% overlays robot on trajectory 

    %% 4.4 Inverse Kinematics
    % Test case (PASS)
    H_test = [cos(pi/4), -sin(pi/4), 0, 20; 
         sin(pi/4), cos(pi/4),  0, 23;
         0,         0,          1, 15;
         0,         0,          0, 1];
    q_test = inverse(H_test,myrobot);
    display(q_test);

    x = linspace(10,30,100);
    y = linspace(23,30,100);
    z = linspace(15,100,100);
    d = [x;y;z];
    R = [cos(pi/4), -sin(pi/4), 0;
         sin(pi/4), cos(pi/4),  0;
         0,         0,          1];

    q = zeros(100,6);
    for i = 1:100
        H = [R, [d(1,i);d(2,i);d(3,i)]; 0, 0, 0, 1];
        q(i,:) = inverse(H, myrobot);
    end
    plot3(x, y, z,'r')
    hold on
    plot(myrobot, q)

end