function Lab3()
    % all units in (mm)
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

    myrobot = mypuma560(DH);

    %% 3.1 Computing Attractive Fields / Tau

    H1 = eul2tr([0 pi pi/2]);
    H1(1:3,4) = 100*[-1;3;3]/4;
    q0 = inverse(H1, myrobot);

    H2 = eul2tr([0 pi -pi/2]);
    H1(1:3,4) = 100*[3;-1;2]/4;
    qf = inverse(H1, myrobot);
    tau = att(q0,qf,myrobot)'

    %% 3.2 Motion Planning (No Obstacles)
    qref = motionplan(q0,qf,0,10,myrobot,[],0.01);
    t=linspace(0,10,300);
    q = ppval(qref,t)';

    for i = 1:300
        full_H = forward_i(6,q(i,:), myrobot);
        o(i,:) = full_H(1:3,4); % just getting us the coordinate column from each H time step
    end
    plot3(o(:,1),o(:,2),o(:,3),'r');
    hold on    
    plot(myrobot,q);

    %% 3.3 Motion Planning (with Obstacles)
    setupobstacle
    q3 = 0.9*q0+0.1*qf;
    tau = rep(q3,myrobot,obs{1})';

    q4 = [pi/2 pi 1.2*pi 0 0 0];
    tau = rep(q4,myrobot,obs{6})';

    hold on
    axis([-100 100 -100 100 0 200])
    view(-32,50)
    plotobstacle(obs);
    qref = motionplan(q0,qf,0,10,myrobot,obs,0.01);
    t=linspace(0,10,300);
    q=ppval(qref,t)';
    for i = 1:300
        full_H = forward_i(6,q(i,:), myrobot);
        o(i,:) = full_H(1:3,4); % just getting us the coordinate column from each H time step
    end
    plot3(o(:,1),o(:,2),o(:,3),'r');
    plot(myrobot,q);

    hold off
end