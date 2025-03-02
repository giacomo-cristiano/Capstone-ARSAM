%%
% Load the robot
robot = importrobot('C:\Users\User\OneDrive - University of Toronto\4th year TERM 1\APS 490 CAPSTONE\Robot Modellig\Mini Assembly R6 Assembly Type\urdf\Mini Assembly R6 Assembly Type.urdf');

% Show robot model
% show(robot);

% Get available body names and trim spaces
bodyNames = strtrim(robot.BodyNames);
disp("Available Bodies:"); disp(bodyNames);

% Get first joint name dynamically
firstJointName = robot.Bodies{1}.Joint.Name;
disp(['First Joint Name: ', firstJointName]);

% Get default joint configuration
q0 = homeConfiguration(robot);

% Find the correct index of the first joint
jointIdx = find(strcmp({q0.JointName}, firstJointName));

% Rotate the joint by 45 degrees
q0(jointIdx).JointPosition = pi/4;

% Show updated robot pose
% show(robot, q0);
% show(robot, q0, 'FastUpdate', true);


%%
% Load the robot
robot = importrobot('C:\Users\User\OneDrive - University of Toronto\4th year TERM 1\APS 490 CAPSTONE\Robot Modellig\Mini Assembly R6 Assembly Type\urdf\Mini Assembly R6 Assembly Type.urdf');

% Show robot model
% show(robot);

% Get available body names and trim spaces
bodyNames = strtrim(robot.BodyNames);
disp("Available Bodies:"); disp(bodyNames);

% Get first joint name dynamically
firstJointName = robot.Bodies{1}.Joint.Name;
disp(['First Joint Name: ', firstJointName]);

% Get default joint configuration
q0 = homeConfiguration(robot);

% Find the correct index of the first joint
jointIdx = find(strcmp({q0.JointName}, firstJointName));

% Animate joint rotation
for theta = 0:0.1:2*pi  % Sweep from 0 to 360 degrees
    q0(jointIdx).JointPosition = theta;  % Update joint position
    show(robot, q0);  % Update visualization
    pause(0.01);  % Control animation speed
end