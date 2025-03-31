function myrobot = mykuka_search(delta)
    % delta is defined as perturbations on (a6, d6) 

    d1 = 400;
    a1 = 25;
    a2 = 315;
    a3 = 35;
    d4 = 365;
    a6 = 296.23;
    d6 = 161.44;

    % DH table (theta, d, a, alpha)
    DH = [0, d1, a1,  pi/2;
          0, 0,  a2,  0;
          0, 0,  a3,  pi/2;
          0, d4, 0,  -pi/2;
          0, 0,  0,   pi/2;
          0, d6 + delta(2), -a6 + delta(1),  0];

    myrobot = SerialLink(DH);
end