function Lab4()
    d1 = 400;
    a1 = 25;
    a2 = 315;
    a3 = 35;
    d4 = 365;
    a6 = 156;
    d6 = 161.44;
    
    % DH table (theta, d, a, alpha)
    DH = [0, d1, a1,  pi/2;
          0, 0,  a2,  0;
          0, 0,  a3,  pi/2;
          0, d4, 0,  -pi/2;
          0, 0,  0,   pi/2;
          0, d6, -a6,  0];
    kuka = mykuka(DH); % intialize robot with gripper

    % DH_forces table (theta, d, a, alpha)
    DH_forces = [0, d1, a1,  pi/2;
          0, 0,  a2,  0;
          0, 0,  a3,  pi/2;
          0, d4, 0,  -pi/2;
          0, 0,  0,   pi/2;
          0, d6, 0,  0];
    kuka_forces = mykuka(DH_forces); % intialize robot

    %% 4.1 Initial Motion Planning
    setupobstacle;
    z_grid = 45;
    p0 = [370 -440 150].';
    p1 = [370 -440 z_grid].';
    p2 = [750 -220 225].';
    p3 = [620 350 225].';
    Rd = [0 0 1; 0 -1 0; 1 0 0];
    
    H0 = [Rd, p0; 0 0 0 1];
    H1 = [Rd, p1; 0 0 0 1];
    H2 = [Rd, p2; 0 0 0 1];
    H3 = [Rd, p3; 0 0 0 1];
    
    q0 = inverse_kuka(H0, kuka);
    q1 = inverse_kuka(H1, kuka);
    q2 = inverse_kuka(H2, kuka);
    q3 = inverse_kuka(H3, kuka)
    
    qref0_1 = motionplan(q0, q1, 0, 10, kuka_forces, obs, 0.01, 0.01, 0.01);
    qref1_2 = motionplan(q1, q2, 0, 10, kuka_forces, obs, 0.01, 0.01, 0.01);
    qref2_3 = motionplan(q2, q3, 0, 10, kuka_forces, obs, 0.01, 0.01, 0.01); 

    % run through planned trajectory in matlab sims
    hold on
    axis([-1000 1000 -1000 1000 0 1000])
    view(-32,50)
    plotobstacle(obs);
    t=linspace(0,10,300);
    
    q=ppval(qref0_1,t)';
    % full_H = forward_i(6,q(1,:), kuka);
    % disp(full_H)
    for i = 1:300 % plots line with robot
        full_H = forward_i(6,q(i,:), kuka);
        o(i,:) = full_H(1:3,4); % just getting us the coordinate column from each H time step
    end
    % disp(full_H)
    plot3(o(:,1),o(:,2),o(:,3),'r');
    plot(kuka,q);

    q=ppval(qref1_2,t)';
    % full_H = forward_i(6,q(1,:), kuka);
    % disp(full_H)
    for i = 1:300 % plots line with robot
        full_H = forward_i(6,q(i,:), kuka);
        o(i,:) = full_H(1:3,4); % just getting us the coordinate column from each H time step
    end
    % disp(full_H)
    plot3(o(:,1),o(:,2),o(:,3),'r');
    plot(kuka,q);

    q=ppval(qref2_3,t)';
    % full_H = forward_i(6,q(1,:), kuka);
    % disp(full_H)
    for i = 1:300 % plots line with robot
        full_H = forward_i(6,q(i,:), kuka);
        o(i,:) = full_H(1:3,4); % just getting us the coordinate column from each H time step
    end
    % disp(full_H)
    plot3(o(:,1),o(:,2),o(:,3),'r');
    plot(kuka,q);

    hold off

    %% 4.2 Initial Motion Planning
    % Base setup for points to travel to
    setupobstacle;
    z_grid = 45;
    p0 = [370 -440 150].';
    p1 = [370 -440 z_grid].';
    p2 = [750 -220 225].';
    p3 = [620 350 225].';
    Rd = [0 0 1; 0 -1 0; 1 0 0];

    H_start = [Rd, p_start]
    H0 = [Rd, p0; 0 0 0 1];
    H1 = [Rd, p1; 0 0 0 1];
    H2 = [Rd, p2; 0 0 0 1];
    H3 = [Rd, p3; 0 0 0 1];
    
    q0 = inverse_kuka(H0, kuka);
    q1 = inverse_kuka(H1, kuka);
    q2 = inverse_kuka(H2, kuka);
    q3 = inverse_kuka(H3, kuka);

    % Find where cylinders are located in real world
    p_cyl1 = [620,0,z_grid].';
    p_cyl2 = [620,-440,z_grid].';
    H_cyl1 = [Rd, p_cyl1; 0 0 0 1 ];
    H_cyl2 = [Rd, p_cyl2; 0 0 0 1 ];
    q_cyl1 = inverse_kuka(H_cyl1,kuka);
    q_cyl2 = inverse_kuka(H_cyl2,kuka);

    % Additional point to avoid cylinders on travelling down from home
    % poistion
    p_start = [330 -350 150].';
    H_start = [Rd, p_start; 0 0 0 1];
    q_start = inverse_kuka(H_start, kuka);

    qref0_1 = motionplan(q0, q1, 0, 10, kuka_forces, obs, 0.01, 0.01, 0.01);
    qref1_2 = motionplan(q1, q2, 0, 10, kuka_forces, obs, 0.01, 0.01, 0.01);
    qref2_3 = motionplan(q2, q3, 0, 10, kuka_forces, obs, 0.01, 0.01, 0.01);

    % Cylinder Center Positions
    % setAngles(q_cyl1, 0.04);
    % setAngles(q_cyl2, 0.04);
    
    % Grab Block
    vel = 0.04;
    setHome(vel);
    setGripper(0); % start open

    setAngles(q_start,0.04)
    setAngles(q0,0.04)
    setAngles(q1,0.04)
    setGripper(1); % move to block and grab

    % Move block between cylinders
    t=linspace(0,10,300);
    q=ppval(qref1_2,t)';
    for i = 1:300
        setAngles(q(i,:),0.04)
    end

    % Move block to basket
    q=ppval(qref2_3,t)';
    for i = 1:300 
        setAngles(q(i,:),0.04)
    end
    setGripper(0); % Deposit block

    %% 4.3 Creative Motion Planning
    % Code picks up two blocks to deposit into basket
    setupobstacle;
    z_grid = 45;
    p0 = [370 -440 150].';
    p1 = [370 -440 z_grid].';
    p2 = [750 -220 225].';
    p3 = [620 350 130].';
    Rd = [0 0 1; 0 -1 0; 1 0 0];
    
    p_cyl1 = [620,0,z_grid].';
    p_cyl2 = [620,-440,z_grid].';
    H_cyl1 = [Rd, p_cyl1; 0 0 0 1 ];
    H_cyl2 = [Rd, p_cyl2; 0 0 0 1 ];
    q_cyl1 = inverse_kuka(H_cyl1,kuka);
    q_cyl2 = inverse_kuka(H_cyl2,kuka);

    p_start = [330 -350 150].';
    H_start = [Rd, p_start; 0 0 0 1];
    q_start = inverse_kuka(H_start, kuka);

    p_btw = [400 350 150].';
    H_btw = [Rd, p_btw; 0 0 0 1];
    q_btw = inverse_kuka(H_btw, kuka);

    p_block2 = [450 -220 z_grid].';
    H_block2 = [Rd, p_block2; 0 0 0 1];
    q_block2 = inverse_kuka(H_block2, kuka);

    H_start = [Rd, p_start]
    H0 = [Rd, p0; 0 0 0 1];
    H1 = [Rd, p1; 0 0 0 1];
    H2 = [Rd, p2; 0 0 0 1];
    H3 = [Rd, p3; 0 0 0 1];
    
    q0 = inverse_kuka(H0, kuka);
    q1 = inverse_kuka(H1, kuka);
    q2 = inverse_kuka(H2, kuka);
    q3 = inverse_kuka(H3, kuka);

    qref0_1 = motionplan(q0, q1, 0, 10, kuka_forces, obs, 0.01, 0.01, 0.01);
    qref1_2 = motionplan(q1, q2, 0, 10, kuka_forces, obs, 0.01, 0.01, 0.01);
    qref2_3 = motionplan(q2, q3, 0, 10, kuka_forces, obs, 0.01, 0.01, 0.01);
    qref3_2btw = motionplan(q3, q_btw, 0, 10, kuka_forces, obs, 0.1, 0.01, 0.01);
    qref2btw_2 = motionplan(q_btw, q2, 0, 10, kuka_forces, obs, 0.01, 0.01, 0.01);

    t=linspace(0,10,300);

    % Cylinder Center Positions
    % setAngles(q_cyl1, 0.04);
    % setAngles(q_cyl2, 0.04);

    % Grab Block
    vel = 0.04;
    setHome(vel);

    setGripper(0); % start open

    % Get First Block
    setAngles(q_start,0.04)
    setAngles(q0,0.04)
    setAngles(q1,0.04)
    setGripper(1);

    % Deposit First Block
    q=ppval(qref1_2,t)';
    for i = 1:300
        setAngles(q(i,:),0.04)
    end
    q=ppval(qref2_3,t)';
    for i = 1:300 
        setAngles(q(i,:),0.04)
    end
    setGripper(0);

    % Go Back Between Cylinders and Get Second Block
    q=ppval(qref3_2btw,t)';
    for i = 1:300 
        setAngles(q(i,:),0.04)
    end
    q=ppval(qref2btw_2,t)';
    for i = 1:300 
        setAngles(q(i,:),0.04)
    end
    setAngles(q_block2,0.04);
    setGripper(1);

    % Deposit Second Block
    q=ppval(qref2_3,t)';
    for i = 1:300 
        setAngles(q(i,:),0.04)
    end
    setGripper(0);

end