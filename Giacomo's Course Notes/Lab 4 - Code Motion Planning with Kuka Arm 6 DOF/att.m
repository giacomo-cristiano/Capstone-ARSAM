function tau = att(q, qf, myrobot)
    zeta = 10^-2;
    d_i = 10^6; % We wil never cross this threshold = 1000m
    tau = 0;
    % qf(4) = qf(4) + 2*pi;

    % forward pass to get H of each joint with respect to base frame
    for i = 1:6
        H_links_init(:,:,i) = forward_i(i,q,myrobot);
        H_links_final(:,:,i) = forward_i(i,qf,myrobot);
    end

    for i = 1:6
        % Get origin positions from H_links
        O_init(:,i) = H_links_init(1:3,4,i);
        O_final(:,i) = H_links_final(1:3,4,i);
        
        O_diff(:,i) = O_init(:,i) - O_final(:,i);
        O_mag = norm(O_diff);

        % Compute attractive forces on each joint
        if (O_mag <= d_i)
            F_att_i(:,i) = -zeta * O_diff(:,i);
        else
            F_att_i(:,i) = -d_i * zeta * (O_diff(:,i) / O_mag);
        end

        % Computer linear velocity Jacobian for each joint (note we only have revolute joints)
        J_i = Jv_i(i,q,myrobot,H_links_init(:,:,:));

        % Compute Tau
        tau = tau + (J_i.' * F_att_i(:,i));
    end

    % disp(F_att_i)

    % Normalize Tau
    if norm(tau) ~= 0
        tau = tau / norm(tau);
    end

end