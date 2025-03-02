robot = importrobot('C:\Users\User\OneDrive - University of Toronto\4th year TERM 1\APS 490 CAPSTONE\Robot Modellig\Mini Assembly R6 Assembly Type\urdf\Mini Assembly R6 Assembly Type.urdf');

% show(robot, 'Visuals', 'on');

% Joint Control 

bodyNames = robot.BodyNames;
bodyNames = strtrim(bodyNames);  % Remove leading/trailing spaces
disp(bodyNames);  % Check cleaned-up names

% Find the index of Joint J1 in the robot model
jointIdx = find(strcmp(robot.BodyNames, 'J1'));  % Find index for J1
disp(['Index of J1: ', num2str(jointIdx)]);

% Rotate link by setting joint position
q0 = homeConfiguration(robot);  % Get default joint configuration
% 
q0.JointPosition = pi/4;  % Rotate Link 1 by 45 degrees

show(robot, q0);  % Update visualization
