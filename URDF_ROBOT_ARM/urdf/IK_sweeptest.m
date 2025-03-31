% % Define sweep grid
% x_vals = linspace(0.2, 0.6,50);   % forward range
% y_vals = linspace(-0.3, 1,50);  % lateral range
% z_vals = linspace(0.1, 0.6, 50);   % vertical range
% 
% % Setup
% enforceJointLimits = true;
% sortByDistance = true;
% referenceConfigstruct = homeConfiguration(robot); % sets 0 0 0 0 0 0 joint angles 
% referenceConfig = [referenceConfigstruct.JointPosition]; % convert to numeric row vector

robot.DataFormat = 'row';  % ✔️ Recommended for sweep tests with numeric configs

% Define sweep grid
x_vals = linspace(0.2, 0.6,50);   % forward range
y_vals = linspace(-0.3, 1,50);    % lateral range
z_vals = linspace(0.1, 0.6, 50);  % vertical range

% Setup
enforceJointLimits = true;
sortByDistance = true;
referenceConfig = homeConfiguration(robot);  % Already a numeric row vector

% ikSol = spray_tip(T, enforceJointLimits, sortByDistance, referenceConfig);

reachable_points = [];  % for plotting
unreachable_points = [];  % for debugging

% Sweep through all (x, y, z) points
for xi = 1:length(x_vals)
    for yi = 1:length(y_vals)
        for zi = 1:length(z_vals)
            pos = [x_vals(xi), y_vals(yi), z_vals(zi)];
            T = trvec2tform(pos);
            
            ikSol = spray_tip(T, enforceJointLimits, sortByDistance, referenceConfig);
            
            if ~isempty(ikSol)
                reachable_points(end+1, :) = pos;
            else
                unreachable_points(end+1, :) = pos;
            end
        end
    end
end

% Save results
save('workspace_reachability.mat', 'reachable_points', 'unreachable_points');

% Plot results
figure;
scatter3(reachable_points(:,1), reachable_points(:,2), reachable_points(:,3), 20, 'g', 'filled');
hold on;
% scatter3(unreachable_points(:,1), unreachable_points(:,2), unreachable_points(:,3), 10, 'r');
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Workspace Reachability Map');
% legend('Reachable', 'Unreachable');
legend('Reachable');
axis equal;
grid on;
