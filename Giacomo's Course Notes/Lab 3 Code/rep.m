function tau_rep = rep(q, myrobot, obs)
    % Computes repulsive torques for a single obstacle.
    % Inputs:
    %   q - Current joint angles (column vector)
    %   myrobot - Robot structure
    %   obs - Single obstacle structure (obs{i})
    % Output:
    %   tau_rep - Repulsive torque vector (normalized)

    % Ensure `obs` is a struct (not a cell)
    if iscell(obs)
        obs = obs{1};
    end

    % Number of links
    num_links = 6;

    % Initialize repulsive torques
    tau_rep = zeros(6,1);

    % Repulsion parameters
    eta = 100;  
    d0 = obs.rho0;  

    % Compute FK for all links
    H_links = zeros(4,4,num_links);
    for i = 1:num_links
        H_links(:,:,i) = forward_link(q, myrobot, i);
    end

    % Loop through each link
    for i = 1:num_links
        oi_q = H_links(1:3, 4, i); % Extract link position (3D vector)

        % Ensure obstacle position `obs.c` is also 3D
        obs_pos = obs.c;
        if length(obs_pos) == 2  % If it's a cylinder, assume z = 0
            obs_pos = [obs_pos; 0];  % Convert to 3D
        end

        % Check dimensions before computing distance
        if length(oi_q) ~= length(obs_pos)
            error("🚨 Dimension mismatch: oi_q is %dx1, obs_pos is %dx1", length(oi_q), length(obs_pos));
        end

        % Compute distance to the obstacle
        dist = norm(oi_q - obs_pos);

        % Compute repulsive force if within influence range
        if dist < d0  
            F_rep = eta * ((1/dist) - (1/d0)) * (1/(dist^2)) * (oi_q - obs_pos) / dist;
        else
            F_rep = zeros(3,1);
        end

        % Compute Jacobian for link i
        J_oi = Jv_i(i, q, myrobot, H_links);

        % Compute torque contribution from link i
        tau_rep = tau_rep + J_oi' * F_rep;
    end

    % Normalize tau_rep so that norm(tau_rep) = 1
    if norm(tau_rep) > 0
        tau_rep = tau_rep / norm(tau_rep);
    end
end
