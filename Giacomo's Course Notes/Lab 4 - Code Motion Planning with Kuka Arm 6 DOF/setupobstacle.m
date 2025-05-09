% Units are centimetres
% Obstacle 1: Plane
obs{1}.rho0 = 150;
obs{1}.h = 32;
obs{1}.type = 'pla';
% Obstacle 2: Cylinder
obs{2}.R = 100;
obs{2}.c = [6.2;0]*100;
obs{2}.rho0 = 150;
obs{2}.h = 572;
obs{2}.type = 'cyl';
% Obstacle 3: Cylinder
obs{3}.R = 100;
obs{3}.c = [6.2;-4.4]*100;
obs{3}.rho0 = 150;
obs{3}.h = 572;
obs{3}.type = 'cyl';
