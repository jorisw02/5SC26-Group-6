addpath('./_functions')
% GOAL: 3 drones starting from different initial positions and take a
% defined formation after hovering.

%% 1. Displacement Based formation control

% Defining graph

w1 = 0.2; % graph weight 1 - 2
w2 = 0.2; % graph weight 2 - 3
w3 = 0.2; % graph weight 3 - 1

% weight matrix
w = [w1+w2;
     w2+w3;
     w3+w1];
D = diag(w);

% adjectancy matrix
A = [0 w1 w2;
     w1 0 w3;
     w2 w3 0];

% Laplacian
L = D-A;
lambda = eig(L);
%%
% Desired positions for drones in formation
x_1 = 0.5; y_1 = 0.5; z_1 = 1.5;
s = 0.5; % stem of traingle
P_des1 = [x_1+0.5;
          y_1;
          z_1];
P_des2 = [x_1-s+0.5;
          y_1-s;
          z_1];
P_des3 = [x_1-s+0.5;
          y_1+s;
          z_1];
P_desired = [P_des1; P_des2; P_des3];

% define dynamics
A = -0.1*eye(length(P_desired),length(P_desired));

%% Simulation displacement based
% Initial Conditions
eps = 0.1;
P1 = [x_1 + eps+0.5;
      y_1 + eps;
      1+2*eps];
P2 = [x_1-s - eps+0.5;
      y_1-s - eps;
      1-eps];
P3 = [x_1-s - 1.5*eps+0.5;
      y_1+s - 1.5*eps;
      1+eps];
IC = [P1;P2;P3];

% time span of simulation
tspan = [0:0.05:150];

[t, p] = ode45(@(t, x) formation_control_position(t,x,A,P_desired),tspan,IC);

%% 
figure(1)
clf
hold on
grid on
view(3)            % Ensures a 3D view
axis equal         % Uniform scaling on all axes

% Plot drone trajectories
plot3(p(:,1), p(:,2), p(:,3), 'b', 'LineWidth', 2);  % p1
plot3(p(:,4), p(:,5), p(:,6), 'r', 'LineWidth', 2);  % p2
plot3(p(:,7), p(:,8), p(:,9), 'g', 'LineWidth', 2);  % p3

% Mark initial positions with blue X
plot3(p(1,1), p(1,2), p(1,3), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
plot3(p(end,1), p(end,2), p(end,3), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
plot3(p(1,4), p(1,5), p(1,6), 'bo', 'MarkerSize', 10, 'LineWidth', 2);
plot3(p(1,7), p(1,8), p(1,9), 'bo', 'MarkerSize', 10, 'LineWidth', 2);

% Mark final positions with red X

plot3(p(end,4), p(end,5), p(end,6), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
plot3(p(end,7), p(end,8), p(end,9), 'rx', 'MarkerSize', 10, 'LineWidth', 2);

% Set axis limits starting from 0
xlim([0, 1.5])
ylim([0, 1.5])
zlim([0.5, 1.5])

% Labels and legend
xlabel('X')
ylabel('Y')
zlabel('Z')
legend('Drone 1','Drone 2','Drone 3', 'Start Position', 'End Position')
title('Equilateral triangle formation')
%% Equilateral triangle

% Coordinates of the drones (vertices of the triangle)
positions = [
    0,   0;   % Drone 1
    1,   0;   % Drone 2
    0,   1;
    1,   1% Drone 3
];

% Define edges of the interaction graph (pairs of vertex indices)
edges = [
    1, 2;
    2, 3;
    3, 4;
    4, 1;
    1,3;
    2,4];

figure
hold on
axis equal
grid on
% Set axis limits
xlim([0 1])
ylim([0 1])

% Plot edges
for i = 1:size(edges,1)
    pt1 = positions(edges(i,1), :);
    pt2 = positions(edges(i,2), :);
    plot([pt1(1), pt2(1)], [pt1(2), pt2(2)], 'k-', 'LineWidth', 1.5);
end

% Plot nodes
scatter(positions(:,1), positions(:,2), 80, 'k', 'filled')

% Title and formatting
%title('Interaction Graph of 3 Drones')
xlabel('X')
ylabel('Y')
