%%
    %% 4.1 Setup
    % all units converted to meters
    a2 = real(sqrt(43.18^2 + 2.03^2));
    d1 = 76;
    d2 = -(38.65 - 15);
    d4 = 43.18;
    d6 = 20;

    % DH table (theta, d, a, alpha)
    DH = [0, d1, 0,  pi/2;
          0, d2, a2, 0;
          0, 0,  0,  pi/2;
          0, d4, 0,  -pi/2;
          0, 0,  0,  pi/2;
          0, d6, 0,  0];

    myrobot = mypuma560(DH); % intialize robot
%%
H1 = eul2tr([0 pi pi/2]);
H1(1:3,4) = 100*[-1; 3; 3]/4;
q1 = inverse(H1, myrobot)';

H2 = eul2tr([0 pi -pi/2]);
H2(1:3,4) = 100*[3; -1; 2]/4;
q2 = inverse(H2, myrobot)';

% Apply the fix:
% q2(4) = q2(4) + 2*pi;

% Compute torques
tau = att(q1, q2, myrobot);
disp("Corrected Torque Vector:");
disp(tau);

%% 3.2 


H1 = eul2tr([0 pi pi/2]); % Convert ZYZ Euler angles
H1(1:3,4) = 100*[-1; 3; 3]/4; 
q0 = inverse(H1, myrobot)'; % Compute joint angles

H2 = eul2tr([0 pi -pi/2]); 
H2(1:3,4) = 100*[3; -1; 2]/4; 
q2 = inverse(H2, myrobot)'; % Compute final joint angles

% Compute trajectory
qref = motionplan(q0, q2, 0, 10, myrobot, [], 0.01);

% Evaluate the trajectory
t = linspace(0, 10, 300);
q = ppval(qref, t)';

% Plot joint angles over time
figure;
plot(t, q);
xlabel("Time (s)");
ylabel("Joint Angles (rad)");
legend("q1", "q2", "q3", "q4", "q5", "q6");
title("Joint Trajectory Using Motion Planning");
grid on;

figure;
plot(myrobot,q);

%%

% Define initial and final poses using homogeneous transformations
H1 = eul2tr([0 pi pi/2]); % Convert Euler angles to transformation matrix
H1(1:3,4) = 100*[-1; 3; 3]/4; 
q0 = inverse(H1, myrobot); % Compute joint angles

H2 = eul2tr([0 pi -pi/2]); 
H2(1:3,4) = 100*[3; -1; 2]/4; 
q2 = inverse(H2, myrobot); % Compute final joint angles

% Load obstacle positions
obs = setupobstacle(); % Function should define obstacles

% Compute trajectory with obstacles
qref = motionplan(q0, q2, 0, 10, myrobot, obs, 0.01);

% Evaluate the trajectory at 300 time steps
t = linspace(0, 10, 300);
q = ppval(qref, t); % Get interpolated joint values

% Plot joint angles over time
plot(t, q);
xlabel("Time (s)");
ylabel("Joint Angles (rad)");
legend("q1", "q2", "q3", "q4", "q5", "q6");
title("Joint Trajectory with Obstacle Avoidance");
grid on;

%% 3.3 
setupobstacle; % Load obstacles

q3 = 0.9*q1 + 0.1*q2; 
% Test with Cylinder (obs{1})
tau_cylinder = rep(q3, myrobot, obs{1});
disp("Repulsive Torque for Cylinder:");
disp(tau_cylinder);

% Test with Sphere (obs{6})
% tau_sphere = rep(q_test, myrobot, obs{6});
% disp("Repulsive Torque for Sphere:");
% disp(tau_sphere);

