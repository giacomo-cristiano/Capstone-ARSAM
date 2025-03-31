function Jv = Jv_i(i, q, myrobot, H_links)
    % Computes the linear Jacobian Jv for the i-th link
    % Inputs:
    %   i - Link index
    %   q - Joint angles
    %   myrobot - Robot model
    %   H_links - All transformation matrices up to each link
    % Output:
    %   Jv - 3x6 Linear Jacobian

    Z_joints = zeros(3,6);
    O_joints = zeros(3,6);
    
    for j = 1:6
        Z_joints(:,j) = H_links(1:3,3,j); % Extract z-axis of each joint
        O_joints(:,j) = H_links(1:3,4,j); % Extract origin of each joint
    end

    Jv = zeros(3,6);
    Jv(:,1) = cross([0;0;1], O_joints(:,i)); % First joint contribution
    
    for j = 2:i
        Jv(:,j) = cross(Z_joints(:,j-1), (O_joints(:,i) - O_joints(:,j-1)));
    end
end
