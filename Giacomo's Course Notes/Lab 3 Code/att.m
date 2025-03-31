function tau = att(q, q2, myrobot)
    % Computes the attractive torques for gradient descent.
    % Inputs:
    %   q  - Current joint angles (column vector)
    %   q2 - Final joint angles (column vector)
    %   myrobot - Robot structure from Lab1.m
    % Outputs:
    %   tau - Normalized torque vector

    % Number of links
    num_links = 6;
    
    % Initialize tau vector
    tau = zeros(6, 1);
    
    % Set zeta value for attraction
    zeta_i = 1;

    % Precompute all link transformations
    H_links = zeros(4,4,num_links);
    for i = 1:num_links
        H_links(:,:,i) = forward_link(q, myrobot, i); % FK up to link i
    end

    % Loop through each link
    for i = 1:num_links
        % Compute forward kinematics for link i (FK stored in H_links)
        oi_q = H_links(1:3, 4, i); % Extract link position

        H_final = forward_link(q2, myrobot, i); % FK up to link i for q2
        oi_q2 = H_final(1:3, 4);

        % Compute attractive force F_att,i(q)
        F_att = -zeta_i * (oi_q - oi_q2) / 100; % Convert to cm scale

        % Compute linear Jacobian for link i
        J_oi = Jv_i(i, q, myrobot, H_links);

        % Compute torque contribution from link i
        tau = tau + J_oi' * F_att;
    end

    % Normalize tau so that norm(tau) = 1
    tau = tau / norm(tau);
end
