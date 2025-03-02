%% BASIC STATIC ROBOT 
% Load the robot
robot = importrobot('C:\Users\User\OneDrive - University of Toronto\Capstone-CODE\Phase 2 ALL LINKS\urdf\Phase 2 ALL LINKS.urdf');

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

% Rotate the joint by 45 degrees
q0(jointIdx).JointPosition = pi/4;

show(robot, q0)

%% Manual JOINT CHECK 

% Load the robot
robot = importrobot('C:\Users\User\OneDrive - University of Toronto\Capstone-CODE\Phase 2 ALL LINKS\urdf\Phase 2 ALL LINKS.urdf');

% Show robot model
% show(robot);

% Get default joint configuration
q0 = homeConfiguration(robot);

% Define six separate joint variables for manual updates
joint1 = pi/4;       % Example: Set manually in the script
joint2 = pi/16;    % Example: 30 degrees
joint3 = -pi/32;   % Example: -30 degrees
joint4 = pi/4;    % Example: 45 degrees
joint5 = pi/8;   % Example: -45 degrees
joint6 = -pi/4;    % Example: 90 degrees

% Store these in an array for easy looping
jointPositions = [joint1, joint2, joint3, joint4, joint5, joint6];

% Loop through the joints and update their positions
for i = 1:min(length(q0), 6)  % Ensure we don't exceed available joints
    disp(['Updating Joint: ', q0(i).JointName]);  % Display joint name
    q0(i).JointPosition = jointPositions(i);  % Assign manual values
end
% overwrites each one 
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
robot = importrobot('C:\Users\User\OneDrive - University of Toronto\Capstone-CODE\Phase 2 ALL LINKS\urdf\Phase 2 ALL LINKS.urdf');

% Show robot model in the existing figure
ax = show(robot); 
hold on;

% Set default isometric camera view
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
pauseDuration = 0.1; % Pause between steps (seconds)

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