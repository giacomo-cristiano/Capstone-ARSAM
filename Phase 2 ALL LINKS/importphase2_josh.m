%% BASIC STATIC ROBOT To launch URDF File
% Load the robot
robot = importrobot('C:\Users\User\Desktop\Capstone-Code\Phase 2 ALL LINKS\urdf\Phase 2 ALL LINKS.urdf');

% Show robot model
% show(robot);

% Get available body names and trim spaces
bodyNames = strtrim(robot.BodyNames);
disp(bodyNames);  % Check cleaned-up names

% Get first joint name dynamically
firstJointName = robot.Bodies{1}.Joint.Name;
disp(['First Joint Name: ', firstJointName]);

% Get default joint configuration
q0 = homeConfiguration(robot);


% Find the correct index of the first joint
jointIdx = find(strcmp({q0.JointName}, firstJointName));

% Rotate the joint by 45 degrees, rotate the base link by 45 degrees, in
% radians
q0(jointIdx).JointPosition = pi/4;
%q0 is the structure of the joint angles, this is what we will be referencing 
show(robot, q0)

%% Manual JOINT CHECK 

% Load the robot
robot = importrobot('C:\Users\User\Desktop\Capstone-Code\Phase 2 ALL LINKS\urdf\Phase 2 ALL LINKS.urdf');

% Show robot model
% show(robot);

% Get default joint configuration, 0 matrix
q0 = homeConfiguration(robot);

% Define six separate joint variables for manual updates, store them in
% array
joint1 = pi/4;       % Example: Set manually in the script
joint2 = pi/16;    % Example: 30 degrees
joint3 = -pi/32;   % Example: -30 degrees
joint4 = pi/4;    % Example: 45 degrees
joint5 = pi/8;   % Example: -45 degrees
joint6 = -pi/4;    % Example: 90 degrees

% Store these in an array for easy looping
jointPositions = [joint1, joint2, joint3, joint4, joint5, joint6];

% Loop through the joints and update their positions, update join positions
% through for loop
for i = 1:min(length(q0), 6)  % Ensure we don't exceed available joints
    disp(['Updating Joint: ', q0(i).JointName]);  % Display joint name
    q0(i).JointPosition = jointPositions(i);  % Assign manual values
end

% Show the updated robot visualization
show(robot, q0);

%% Sequential Joint Movement with Incremental Steps (Reuses Same Figure)

% Check if figure exists, otherwise create it
figTag = 'robot_simulation_figure';  % Unique figure tag
figHandle = findobj('Type', 'Figure', 'Name', figTag);  % Find existing figure

if isempty(figHandle)

    figHandle = figure('Name', figTag); % Create a new figure only if it doesn't exist
else
    figure(figHandle); % Bring existing figure to focus
    clf(figHandle); % Clear figure to reset visualization
    pause(0.1);
end

% Load the robot
robot = importrobot('C:\Users\User\Desktop\Capstone-Code\Phase 2 ALL LINKS\urdf\Phase 2 ALL LINKS.urdf');

% Show robot model in the existing figure
ax = show(robot); 
hold on;

% Set default isometric camera view, setting up the camera
view(3);  % Isometric 3D view
campos([1, 1, 1] * 1.3); % Set camera position (closer to the robot)
camtarget([0, 0, 0]); % Center camera target near the base of the arm
camva(20); % Adjust zoom level (smaller = more zoomed in)

drawnow; % Force immediate rendering

% Get default joint configuration
q0 = homeConfiguration(robot);

% Define final joint positions (target angles)
jointTargets = [pi/2, pi/4, pi/4, pi/4, pi/4, pi/4]; % First joint = 90 degrees (pi/2)

% Define the number of increments for smooth motion
numSteps = 20; % Adjust for finer or coarser movement
pauseDuration = 0.005; % Pause between steps (seconds)

% Loop through each joint and incrementally update its position
for i = 1:min(length(q0), 6)  % Ensure we don't exceed available joints
    disp(['Updating Joint: ', q0(i).JointName]);  % Display joint name
    
    % Reset all joints to home position
    qCurrent = homeConfiguration(robot);
    
    % Gradually move the joint in small increments
    for step = 1:numSteps
        qCurrent(i).JointPosition = (step / numSteps) * jointTargets(i); % Incremental step updates up to setpoints
        show(robot, qCurrent, 'PreservePlot', false, 'Parent', ax); % displays newly computed positon
        pause(pauseDuration); % Small delay to visualize movement
    end
end

%% To DO
%SIMSCAPE to make the joint pathways in simscape - Inverse Kinematics and
%Forward kinematics through SIMSCAPE, this is your task. how to get unity
%to work with simscape
smimport('C:\Users\User\Desktop\Capstone-Code\Phase 2 ALL LINKS\urdf\Phase 2 ALL LINKS.urdf')


%% Import STL File
stlPath = 'C:\Users\joshu\Downloads\Capstone-CODE-main\naca 0015.stl'; % Define file path
fv = stlread(stlPath); % Read STL file

figure;
trisurf(fv.ConnectivityList, fv.Points(:,1), fv.Points(:,2), fv.Points(:,3), ...
    'FaceColor', 'cyan', 'EdgeColor', 'none'); % Render STL as a surface
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('STL Model: NACA 0015');
camlight; lighting phong;

%% Coordinate Mapping Triangle

vertices = fv.Points;  % Extract unique vertex coordinates
faces = fv.ConnectivityList; % Extract face connectivity (triangles)
faceCenters = (vertices(faces(:,1), :) + vertices(faces(:,2), :) + vertices(faces(:,3), :)) / 3;

figure;
trisurf(faces, vertices(:,1), vertices(:,2), vertices(:,3), ...
    'FaceColor', 'cyan', 'EdgeColor', 'black', 'FaceAlpha', 0.7);
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('STL Surface with Face Centers');
grid on;
view(3); % Ensure a 3D view

%% Load STL File
stlPath = 'C:\Users\joshu\Downloads\Capstone-CODE-main\naca 0015.stl';
fv = stlread(stlPath);

%% Extract Vertices and Faces
vertices = fv.Points;  % Extract unique vertex coordinates
faces = fv.ConnectivityList; % Extract face connectivity (triangles)

%% Define a 90-degree Rotation Matrix for Y-axis
theta_y = pi/2; % 90 degrees in radians
Ry = [cos(theta_y)  0  sin(theta_y);
      0            1  0;
     -sin(theta_y) 0  cos(theta_y)];

%% Apply Rotation to All Vertices
rotatedVertices = (Ry * vertices')'; 

%% Shift Shape Up so the Lowest Z = 0
minZ = min(rotatedVertices(:,3)); % Find the lowest Z-value
rotatedVertices(:,3) = rotatedVertices(:,3) - minZ; % Shift up

%% Identify and Remove Points Where Z = 0
z_plane_indices = abs(rotatedVertices(:,3)) < 1e-6; % Small threshold for numerical precision
valid_vertex_indices = find(~z_plane_indices); % Keep only non-Z=0 points

% Create a mapping from old vertex indices to new indices
new_index_map = zeros(size(vertices,1), 1);
new_index_map(valid_vertex_indices) = 1:length(valid_vertex_indices); % Assign new indices

% Filter out faces that reference removed vertices
valid_faces_mask = all(ismember(faces, valid_vertex_indices), 2);
filtered_faces = faces(valid_faces_mask, :);
filtered_faces = new_index_map(filtered_faces); % Update face indices

% Get the filtered vertices after removing Z=0 points
filtered_vertices = rotatedVertices(valid_vertex_indices, :);

%% Compute New Face Centers After Removing Z = 0 Points
faceCenters = (filtered_vertices(filtered_faces(:,1), :) + ...
               filtered_vertices(filtered_faces(:,2), :) + ...
               filtered_vertices(filtered_faces(:,3), :)) / 3;

%% Plot Filtered and Shifted STL
figure;
trisurf(filtered_faces, filtered_vertices(:,1), filtered_vertices(:,2), filtered_vertices(:,3), ...
    'FaceColor', 'cyan', 'EdgeColor', 'black', 'FaceAlpha', 0.7);
hold on;
scatter3(faceCenters(:,1), faceCenters(:,2), faceCenters(:,3), 20, 'red', 'filled'); % Face centers in red
hold off;

axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Rotated & Shifted STL (Lowest Z = 0, Removed Z = 0)');
grid on;
view(3); % Ensure a 3D view

%%%%%%%%%%%%%%%%%%%


%% Load STL File QUADRILATERAL MESH
stlPath = 'C:\Users\joshu\Downloads\Capstone-CODE-main\naca 0015.stl';
fv = stlread(stlPath);

%% Extract Vertices and Faces
vertices = fv.Points;
faces = fv.ConnectivityList;

%% Define a 90-degree Rotation Matrix for Y-axis
theta_y = pi/2; % 90 degrees in radians
Ry = [cos(theta_y)  0  sin(theta_y);
      0            1  0;
     -sin(theta_y) 0  cos(theta_y)];

%% Apply Rotation to All Vertices
rotatedVertices = (Ry * vertices')'; 

%% Shift Shape Up so the Lowest Z = 0
minZ = min(rotatedVertices(:,3)); % Find the lowest Z-value
rotatedVertices(:,3) = rotatedVertices(:,3) - minZ; % Shift up

%% Identify and Remove Points Where Z = 0
z_plane_indices = abs(rotatedVertices(:,3)) < 1e-6; % Small threshold for numerical precision
valid_vertex_indices = find(~z_plane_indices); % Keep only non-Z=0 points

% Create a mapping from old vertex indices to new indices
new_index_map = zeros(size(vertices,1), 1);
new_index_map(valid_vertex_indices) = 1:length(valid_vertex_indices); % Assign new indices

% Filter out faces that reference removed vertices
valid_faces_mask = all(ismember(faces, valid_vertex_indices), 2);
filtered_faces = faces(valid_faces_mask, :);
filtered_faces = new_index_map(filtered_faces); % Update face indices

% Get the filtered vertices after removing Z=0 points
filtered_vertices = rotatedVertices(valid_vertex_indices, :);

%% Generate a Coarser Quadrilateral Mesh (Larger Squares)
gridResolution = 10; % Lower value = larger squares

xRange = linspace(min(filtered_vertices(:,1)), max(filtered_vertices(:,1)), gridResolution);
yRange = linspace(min(filtered_vertices(:,2)), max(filtered_vertices(:,2)), gridResolution);
[Xq, Yq] = meshgrid(xRange, yRange); % Coarse quadrilateral mesh grid

% Interpolate Z-values with 'linear' method for less distortion
F = scatteredInterpolant(filtered_vertices(:,1), filtered_vertices(:,2), filtered_vertices(:,3), 'linear');
Zq = F(Xq, Yq); % Compute new Z-values for a smoother quadrilateral surface

%% Compute Centers of Each Square in the Mesh
Xc = (Xq(1:end-1,1:end-1) + Xq(2:end,1:end-1) + Xq(1:end-1,2:end) + Xq(2:end,2:end)) / 4;
Yc = (Yq(1:end-1,1:end-1) + Yq(2:end,1:end-1) + Yq(1:end-1,2:end) + Yq(2:end,2:end)) / 4;
Zc = (Zq(1:end-1,1:end-1) + Zq(2:end,1:end-1) + Zq(1:end-1,2:end) + Zq(2:end,2:end)) / 4;

%% Compute Surface Normals at Each Square Center
[rows, cols] = size(Xc);
normals = zeros(rows, cols, 3); % Allocate space for normals

for i = 1:rows-1
    for j = 1:cols-1
        % Define four surrounding points (forming the quadrilateral)
        p1 = [Xc(i,j), Yc(i,j), Zc(i,j)];
        p2 = [Xc(i+1,j), Yc(i+1,j), Zc(i+1,j)];
        p3 = [Xc(i,j+1), Yc(i,j+1), Zc(i,j+1)];
        
        % Compute vectors along two edges
        v1 = p2 - p1;
        v2 = p3 - p1;
        
        % Compute normal as cross product of edge vectors
        n = cross(v1, v2);
        n = n / norm(n); % Normalize to unit length
        normals(i, j, :) = n; % Store normal vector
    end
end

%% Plot the Coarse 3D Quadrilateral Mesh with Normals
figure;
hold on;

% Plot the quadrilateral mesh
surf(Xq, Yq, Zq, 'FaceColor', 'cyan', 'EdgeColor', 'black');

% Mark the centers of each square with red dots
scatter3(Xc(:), Yc(:), Zc(:), 30, 'red', 'filled'); % Mark square centers

% Extract normal components correctly for plotting
Nx = reshape(normals(:,:,1), [], 1); % X component
Ny = reshape(normals(:,:,2), [], 1); % Y component
Nz = reshape(normals(:,:,3), [], 1); % Z component

% Plot normals at each square center
quiver3(Xc(:), Yc(:), Zc(:), Nx, Ny, Nz, 0.5, 'k', 'LineWidth', 1.5);

hold off;
axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('3D Quadrilateral (Square) Mesh with Surface Normals');
grid on;
view(3); % Ensure a 3D visualization


%%%% INVERSE KINEMATICS REVIEW

%% Load the Robot Model
robot = importrobot('C:\Users\User\Desktop\Capstone-Code\Phase 2 ALL LINKS\urdf\Phase 2 ALL LINKS.urdf');
robot.DataFormat = 'row';  

% Show initial robot position with collision visualization
figure;
ax = show(robot, 'Collisions', 'on'); 
title('Initial Robot Configuration with Collision Display');
grid on; view(3);

%% Add Collision Objects to Robot Links
% for i = 1:length(robot.Bodies)
%     link = robot.Bodies{i};
% 
%     % Assign a default collision box if none exists
%     if isempty(link.Collisions)
%         boxObj = collisionBox(0.05, 0.05, 0.05); % Adjust size
%         tform = eye(4); % No transformation needed
%         addCollision(robot, link.Name, boxObj, tform);
%     end
% end
robot2 = importrobot('C:\Users\User\Desktop\Capstone-Code\Phase 2 ALL LINKS\urdf\workingrobot.urdf');
figure;
ax = show(robot2, 'Collisions', 'on'); 
title('Initial Robot Configuration with Collision Display');
grid on; view(3);
%% Define Inverse Kinematics Solver
ik = inverseKinematics('RigidBodyTree', robot);
weights = [1 1 1 1 1 1]; % XYZ + Orientation priority
endEffector = robot.BodyNames{end}; % Get the last link (end-effector)
qStart = randomConfiguration(robot); % Use random valid pose

%% Define the Target Pose (Modify to Give More Space)
targetPos = [0.5, 0.3, 0.5]; % Adjusted to give more reach space
targetRPY = [0, pi/4, 0]; % Roll-Pitch-Yaw angles
targetRotm = eul2rotm(targetRPY, 'XYZ'); % Convert Euler angles to Rotation Matrix

% Create Homogeneous Transformation Matrix
targetPose = eye(4);
targetPose(1:3, 1:3) = targetRotm; % Set rotation
targetPose(1:3, 4) = targetPos'; % Set position (Ensure column vector format)

%% Solve IK to Get a Valid Target Configuration
maxAttempts = 15; % Retry IK multiple times if collision occurs
attempt = 1;
collisionDetected = true;

while attempt <= maxAttempts && collisionDetected
    [qGoal, solInfo] = ik(endEffector, targetPose, weights, qStart);

    % Ensure IK solution is valid
    if isempty(qGoal) || any(isnan(qGoal)) || any(isinf(qGoal))
        disp('❌ IK solution invalid, retrying...');
        qStart = randomConfiguration(robot); % Try a new starting pose
        attempt = attempt + 1;
        continue;
    end

    % Check if the goal configuration causes collisions (Fix: Removed SeparationThreshold)
    [collisionDetected, collisionPairs] = checkCollision(robot, qGoal, "SkippedSelfCollisions", "adjacent");

    % Ensure collisionPairs is a valid matrix
    collisionPairs(isnan(collisionPairs) | isinf(collisionPairs)) = 0; % Remove invalid values

    if collisionDetected
        disp(['⚠️ Collision detected at IK solution (Attempt ', num2str(attempt), ')']);
        disp('Colliding Links (Filtered):');
        disp(collisionPairs); % Show only valid collision data

        % Slightly modify target position to escape collision
        targetPose(1:3, 4) = targetPos' + (rand(3,1) - 0.5) * 0.02;

        % Try a new starting pose
        qStart = randomConfiguration(robot);
        attempt = attempt + 1;
    else
        disp('✅ Collision-free IK solution found!');
    end
end

if collisionDetected
    error('❌ Could not find a collision-free IK solution after multiple attempts.');
end

%% Show IK Solution to Debug Before Planning
figure;
show(robot, qGoal, 'Collisions', 'on');
title('Checking IK Solution Before Motion Planning');
grid on; view(3);

%% Define Motion Planner (RRT for Collision-Free Path)
planner = manipulatorRRT(robot, collisionObjects(robot));

% Configure the planner
planner.MaxConnectionDistance = 0.05; % Reduce step size for better accuracy
planner.ValidationDistance = 0.01; % Higher accuracy for collision checking

% Plan a collision-free path
path = plan(planner, qStart, qGoal); 

% Check if a valid path was found
if isempty(path)
    error('❌ No collision-free path found using RRT.');
end

disp('✅ Collision-free path found!');

%% Animate the Robot Following the Collision-Free Path
%% Apply Joint Limits Based on Your Constraints
robot.Bodies{1}.Joint.PositionLimits = [-pi, pi];      % Joint 1: 360° unrestricted (or limit to 180° for spraying)
robot.Bodies{2}.Joint.PositionLimits = [-pi/4, pi/4];  % Joint 2: -45° to 45°
robot.Bodies{3}.Joint.PositionLimits = [-pi/6, pi/2];  % Joint 3: -30° to 90°
robot.Bodies{4}.Joint.PositionLimits = [-pi/2, pi/2];  % Joint 4: -90° to 90°
robot.Bodies{5}.Joint.PositionLimits = [-pi/3, pi/3];  % Joint 5: -60° to 60°
robot.Bodies{6}.Joint.PositionLimits = [-pi/6, pi/6];  % Joint 6: -30° to 30°
%% Check If IK Solution Respects Joint Limits
function valid = isValidJointConfiguration(robot, qGoal)
    for i = 1:length(qGoal)
        limits = robot.Bodies{i}.Joint.PositionLimits;
        if qGoal(i) < limits(1) || qGoal(i) > limits(2)
            valid = false;
            return;
        end
    end
    valid = true;
end
maxAttempts = 20; % Retry IK multiple times if collision occurs
attempt = 1;
collisionDetected = true;

while attempt <= maxAttempts && (collisionDetected || ~isValidJointConfiguration(robot, qGoal))
    [qGoal, solInfo] = ik(endEffector, targetPose, weights, qStart);

    if isempty(qGoal) || any(isnan(qGoal)) || any(isinf(qGoal))
        disp('❌ No valid IK solution found, retrying...');
        qStart = randomConfiguration(robot); % Try a new starting pose
        attempt = attempt + 1;
        continue;
    end

    % Check for Collisions
    collisionDetected = checkCollision(robot, qGoal, "SkippedSelfCollisions", "adjacent");

    if collisionDetected || ~isValidJointConfiguration(robot, qGoal)
        disp(['⚠️ Collision or joint limit exceeded (Attempt ', num2str(attempt), ')']);
        
        % Move target slightly in random small increments
        targetPose(1:3, 4) = targetPos' + (rand(3,1) - 0.5) * 0.05;

        % Try a new starting pose
        qStart = randomConfiguration(robot);
        attempt = attempt + 1;
    else
        disp('✅ Collision-free IK solution within joint limits found!');
    end
end

if collisionDetected
    error('❌ Could not find a collision-free IK solution within joint limits after multiple attempts. Try a different target position.');
end


