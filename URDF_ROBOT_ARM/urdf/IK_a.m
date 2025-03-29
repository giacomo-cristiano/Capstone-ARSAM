%% 
% Load your robot
robot = importrobot('C:\Users\User\Desktop\Capstone-Code\URDF_ROBOT_ARM\urdf\URDF_ROBOT_ARM.urdf');
robot.DataFormat = 'row';

% Create the analytical IK solver
aik = analyticalInverseKinematics(robot);

% Show supported end-effectors (usually last link in chain)
ikGroups = aik.showdetails();

ikFcn = generateIKFunction(aik, 'spray_tip');
