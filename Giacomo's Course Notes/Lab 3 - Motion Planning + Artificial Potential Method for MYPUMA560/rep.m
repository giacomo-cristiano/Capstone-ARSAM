function tau = rep(q, myrobot, obs)
    n_i = 10^6;
    tau = zeros(6,1);

    % forward pass to get H of each joint with respect to base frame
    for i = 1:6
        H_links_init(:,:,i) = forward_i(i,q,myrobot);
    end

    for i = 1:6
        % Calculate O_i - b and corresponding magnitude
        O_init(:,i) = H_links_init(1:3,4,i);
        
        if obs.type == 'sph'
            center_dist = O_init(:,i) - obs.c;
            b = obs.c + obs.R * (center_dist)/norm(center_dist);
        elseif obs.type == 'cyl'
            center_dist = O_init(1:2,i) - obs.c;
            b = [obs.c + obs.R * (center_dist)/norm(center_dist); O_init(3,i)];
        end

        p_0 = norm(O_init(:,i) - b);
        grad = (O_init(:,i) - b)/p_0;

        % Compute repulsive forces on each joint
        if p_0 <= obs.rho0
            F_rep_i(:,i) = n_i * (1/p_0 - 1/obs.rho0) * (1/p_0^2) * grad;
        else
            F_rep_i(:,i) = zeros(3,1);
        end

        % Computer linear velocity Jacobian for each joint (note we only have revolute joints)
        J_i = Jv_i(i,q,myrobot,H_links_init(:,:,:));

        % Compute Tau
        tau = tau + (J_i.' * F_rep_i(:,i));
    end

    % disp(F_rep_i)

    % Normalize Tau
    if norm(tau) ~= 0
        tau = tau / norm(tau);
    end

end