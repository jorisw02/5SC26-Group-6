addpath('./_functions')
% GOAL: 3 drones starting from different initial positions and take a
% defined formation after hovering.

%% 1. Displacement Based formation control

% Defining graph

w1 = 0.2; % graph weight 1 - 2
w2 = 0.2; % graph weight 2 - 3
w3 = 0.2; % graph weight 3 - 1

% d1 = 0.5; % distance between 1 - 2 in [m]
% d2 = 0.5; % distance between 2 - 3 in [m]
% d3 = 0.5; % distance between 3 - 1 in [m]

% weight matrix
w = [w1;
     w2;
     w3];
W = diag(w);

% incidence matrix
H = eye(length(W),length(W));
H(end,1) = -1;
for i = 1:length(H)-1
    H(i,i+1) = -1;
end

% Laplacian
L = transpose(H)*W*H;
lambda = eig(L);

% Desired positions for drones in formation
x_1 = 0.5; y_1 = 0.5; z_1 = 1.5;
s = 0.5; % stem of traingle
P_des1 = [x_1;
          y_1;
          z_1];
P_des2 = [x_1-s;
          y_1-s;
          z_1];
P_des3 = [x_1-s;
          y_1+s;
          z_1];
P_desired = [P_des1; P_des2; P_des3];

%% Simulation displacement based
% Initial Conditions
eps = 0.1;
P1 = [x_1 + eps;
      y_1 + eps;
      1];
P2 = [x_1-s - eps;
      y_1-s - eps;
      1-0.2];
P3 = [x_1-s - 1.5*eps;
      y_1+s - 1.5*eps;
      1+0.2];
IC = [P1;P2;P3];

% time span of simulation
tspan = [0:0.05:15];

[t, p] = ode45(@(t, x) formation_control_displacement(t,x,L,P_desired),tspan,IC);


%% Plotting
figure(1)
xlabel('X'); ylabel('Y'); zlabel('Z');
hold on; grid on;

% Trajectories
plot3(p(:,1), p(:,2), p(:,3), 'r', 'LineWidth', 1.5)   % Drone 1
plot3(p(:,1+3), p(:,2+3), p(:,3+3), 'g', 'LineWidth', 1.5) % Drone 2
plot3(p(:,1+6), p(:,2+6), p(:,3+6), 'b', 'LineWidth', 1.5) % Drone 3

h_start = plot3(nan, nan, nan, 'o', 'Color', 'b', 'LineWidth', 1.5);      
h_end   = plot3(nan, nan, nan, 'x', 'Color', 'r', 'LineWidth', 1.5);      

% Initial positions
plot3(p(1,1), p(1,2), p(1,3), 'o', 'Color', 'b', 'LineWidth', 1.5, ...
      'MarkerFaceColor', 'none', 'HandleVisibility','off')
plot3(p(1,1+3), p(1,2+3), p(1,3+3), 'o', 'Color', 'b', 'LineWidth', 1.5, ...
      'MarkerFaceColor', 'none', 'HandleVisibility','off')
plot3(p(1,1+6), p(1,2+6), p(1,3+6), 'o', 'Color', 'b', 'LineWidth', 1.5, ...
      'MarkerFaceColor', 'none', 'HandleVisibility','off')

% Final positions
plot3(p(end,1), p(end,2), p(end,3), 'x', 'Color', 'r', 'LineWidth', 1.5, ...
      'MarkerSize', 8, 'HandleVisibility','off')
plot3(p(end,1+3), p(end,2+3), p(end,3+3), 'x', 'Color', 'r', 'LineWidth', 1.5, ...
      'MarkerSize', 8, 'HandleVisibility','off')
plot3(p(end,1+6), p(end,2+6), p(end,3+6), 'x', 'Color', 'r', 'LineWidth', 1.5, ...
      'MarkerSize', 8, 'HandleVisibility','off')

% Legend
legend({'Drone 1', 'Drone 2', 'Drone 3', ...
        'Start Position', 'End Position'});

title('Equilateral Triangle Formation');
view(3);
