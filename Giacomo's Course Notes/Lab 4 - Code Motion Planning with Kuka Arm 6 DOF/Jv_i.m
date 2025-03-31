function Jv = Jv_i(i,q, myrobot, H_links)
    Z_joints = zeros(3,6);
    O_joints = zeros(3,6);
    for j = 1:6 % extract z0_j and O0_j from all H links for each joint j
        Z_joints(:,j) = H_links(1:3,3,j);
        O_joints(:,j) = H_links(1:3,4,j);
    end

    Jv = zeros(3,6);
    % calculate linear velocity jacobian using formulas
    Jv(:,1) = cross([0;0;1], O_joints(:,i));
    for j = 2:i % note we only have revolute joints
        Jv(:,j) = cross(Z_joints(:,j-1), (O_joints(:,i) - O_joints(:,j-1)));
    end
end