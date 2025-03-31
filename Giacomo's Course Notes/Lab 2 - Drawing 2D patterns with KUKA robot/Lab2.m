function Lab2()
    %% 3.0 PreLab
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
          0, d6, -a6,  0];

    kuka = mykuka(DH); % intialize robot

    q_test = [pi/5; pi/3; -pi/4; pi/4; pi/3; pi/4].';
    H_for = forward_kuka(kuka,q_test);
    q_inv = inverse_kuka(H_for,kuka);
    
    %% 4.3 Calibration of DH params
    delta = fminunc(@deltajoint, [0,0]); %   -2.2207  -10.6655
    kuka = mykuka_search(delta);

    R = [0 0  1;
         0 -1 0;
         1 0  0];
    X1 = [0,0,0].'; % sample point
    H_verify = [R X1;
                0,0,0,1];
    q = inverse_kuka(H_verify,kuka)
    setangles(q,0.04);

    %% 4.4 Workspace vs Base Frame

    p_workspace = [600; 100; 10];
    p_baseframe = FrameTransformation(p_workspace);
    R = [0 0  1;
         0 -1 0;
         1 0  0];
    H_frame = [R p_baseframe;
                0,0,0,1];
    q = inverse_kuka(H_frame,kuka)
    setangles(q,0.04);

    %% 4.5 Drawings
    % mysegment();
    % mycircle();
    % myjug();
    % mypattern();
    mysponge();
end