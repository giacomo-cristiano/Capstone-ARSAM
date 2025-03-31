function qref = motionplan(q0, q2, t1, t2, myrobot, obs, tol)
    % Implements motion planning using gradient descent
    % Inputs:
    %   q0 - Initial joint angles (column vector)
    %   q2 - Final joint angles (column vector)
    %   t1 - Start time of trajectory
    %   t2 - End time of trajectory
    %   myrobot - Robot structure
    %   obs - Obstacle structure (not used here)
    %   tol - Tolerance for termination
    % Output:
    %   qref - Piecewise cubic polynomial trajectory
    % Initialize parameters
    alpha = 0.01;  % Learning rate (step size)
    max_iters = 5000; % Maximum iterations
    q = q0'; % Store waypoints as rows (N x 6 matrix)
    
    % Gradient descent loop
    for iter = 1:max_iters
        tau_att = att(q(end, :)', q2, myrobot); % Compute attractive torques
        tau = tau_att'; % No repulsive forces in this section
        
        % compute repulsive torques
        tau_rep = rep(q(end, :)', myrobot, obs);
        
        tau = tau_att + tau_rep; 
        % Update q using gradient descent
        q_new = q(end, :) + alpha * tau;
        q = [q; q_new]; % Append new waypoint

        % Check termination condition (ignore q6)
        if norm(q(end,1:5) - q2(1:5)') < tol
            break;
        end
    end

    % Generate interpolated cubic spline trajectory
    t = linspace(t1, t2, size(q,1));
    qref = spline(t, q'); % qref defines joint angles as a function of time
end
