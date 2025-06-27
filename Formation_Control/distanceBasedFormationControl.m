%% Gradient control law
% The aim is to write a control law in order to obtain a triangle shaped
% formation (the latter is choosen since the resultant graph is "automatically rigid")

%% Agents modeling
% All the drones are represented as single integrator

% Simulation set up
dt = 0.05;
T = 15;
Nsteps = T/dt;

% Initial positions (randomly choosen)
p1 = [0; 0; 0];
p2 = [1.5; 0.5; 0];
p3 = [0.5; 1.5; 0];

positions = [p1, p2, p3];

% Trajectories initialization
traj1 = zeros(3, Nsteps);
traj2 = zeros(3, Nsteps);
traj3 = zeros(3, Nsteps);

% Initial condition setting
traj1(:,1) = positions(:,1);
traj2(:,1) = positions(:,2);
traj3(:,1) = positions(:,3);

%% Desired shape definition
% As mentiond before the desired shape is an equilateral triangle of
% side equal to 0.5m

s = 0.5;

d12 = s;
d23 = s;
d31 = s;

%% Interaction graph construction
% This graph point out with which agent interacts one. In this case
% everybody interacts with everybody.

edges = [1 2; 2 3; 3 1];

% Desired distances vector

D_desired = [d12; d23; d31];

%% Potential function calculation
% % Matrix cointaining all the the desired distances
% 
d_star = zeros(3);

d_star(1,2) = d12;
d_star(2,1) = d12;

d_star(2,3) = d23;
d_star(3,2) = d23;

d_star(3,1) = d31;
d_star(1,3) = d31;

kP = 1;
% 
% V1 = potentialFunction(1, positions, edges, d_star, kP);
% V2 = potentialFunction(2, positions, edges, d_star, kP);
% V3 = potentialFunction(3, positions, edges, d_star, kP);
% 
% % Tourn out to be useless :( I could have directly 
% % calculate the controllaw for each drone

%% Control Action calculation

u1 = controlLaw(1, positions, edges, d_star, kP);
u2 = controlLaw(2, positions, edges, d_star, kP);
u3 = controlLaw(3, positions, edges, d_star, kP);

%% Simulation loop
for t = 1:Nsteps-1
    % Compute control inputs
    u1 = controlLaw(1, positions, edges, d_star, kP);
    u2 = controlLaw(2, positions, edges, d_star, kP);
    u3 = controlLaw(3, positions, edges, d_star, kP);

    % Update positions using Euler integration
    positions(:,1) = positions(:,1) + dt * u1;
    positions(:,2) = positions(:,2) + dt * u2;
    positions(:,3) = positions(:,3) + dt * u3;

    % Save positions to trajectories
    traj1(:,t+1) = positions(:,1);
    traj2(:,t+1) = positions(:,2);
    traj3(:,t+1) = positions(:,3);
end

%% Plotting the 3D trajectories
% figure;
% plot3(traj1(1,:), traj1(2,:), traj1(3,:), 'r', 'LineWidth', 1.5); hold on;
% plot3(traj2(1,:), traj2(2,:), traj2(3,:), 'g', 'LineWidth', 1.5);
% plot3(traj3(1,:), traj3(2,:), traj3(3,:), 'b', 'LineWidth', 1.5);
% 
% % Final positions
% scatter3(traj1(1,end), traj1(2,end), traj1(3,end), 80, 'r', 'filled');
% scatter3(traj2(1,end), traj2(2,end), traj2(3,end), 80, 'g', 'filled');
% scatter3(traj3(1,end), traj3(2,end), traj3(3,end), 80, 'b', 'filled');
% 
% grid on; axis equal;    
% xlabel('X'); ylabel('Y'); zlabel('Z');
% title('Equilateral Triangle Formation');
% legend('Drone 1', 'Drone 2', 'Drone 3');
% view(3);
%% Plotting the 3D trajectories
figure;
plot3(traj1(1,:), traj1(2,:), traj1(3,:), 'r', 'LineWidth', 1.5); hold on;
plot3(traj2(1,:), traj2(2,:), traj2(3,:), 'g', 'LineWidth', 1.5);
plot3(traj3(1,:), traj3(2,:), traj3(3,:), 'b', 'LineWidth', 1.5);

% Initial positions (empty blue circles)
scatter3(traj1(1,1), traj1(2,1), traj1(3,1), 80, 'b', 'o', 'LineWidth', 1.5);
scatter3(traj1(1,end), traj1(2,end), traj1(3,end), 80, 'r', 'x', 'LineWidth', 1.5);
scatter3(traj2(1,1), traj2(2,1), traj2(3,1), 80, 'b', 'o', 'LineWidth', 1.5);
scatter3(traj3(1,1), traj3(2,1), traj3(3,1), 80, 'b', 'o', 'LineWidth', 1.5);

% Final positions (x markers)
scatter3(traj1(1,end), traj1(2,end), traj1(3,end), 80, 'r', 'x', 'LineWidth', 1.5);
scatter3(traj2(1,end), traj2(2,end), traj2(3,end), 80, 'r', 'x', 'LineWidth', 1.5);
scatter3(traj3(1,end), traj3(2,end), traj3(3,end), 80, 'r', 'x', 'LineWidth', 1.5);

grid on; axis equal;    
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Equilateral Triangle Formation');
legend('Drone 1', 'Drone 2', 'Drone 3', 'Start Position', 'End Position');
view(3);

%% Graph plotting
nodes = 1:3;
edges = [1 2; 2 3; 3 1];  

G = graph(edges(:,1), edges(:,2));

nodeX = [0, 1, 0.5];  % X
nodeY = [0, 0, sqrt(3)/2];  % Y 

figure;
h = plot(G, 'XData', nodeX, 'YData', nodeY, ...
         'NodeColor', 'k', 'MarkerSize', 12, ...
         'NodeLabelColor', 'w', 'LineWidth', 2);

title('Interaction Graph of 3 Drones');
axis equal;
