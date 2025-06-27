addpath('./_functions')
%% Simulation parameters
dt = 0.05;
T = 15;
Nsteps = T/dt;

%% Initial positions (random)
p1 = [0; 0; 0];
p2 = [2; 0; 0];
p3 = [2; 2; 0];
p4 = [0; 2; 0];

positions = [p1, p2, p3, p4];

% Trajectories initialization
traj = zeros(3, Nsteps, 4);
for i = 1:4
    traj(:,1,i) = positions(:,i);
end

%% Desired distances
s = 1;  % side of the square
d12 = s;
d23 = s;
d34 = s;
d41 = s;
d13 = sqrt(2)*s;  % diagonal (rigidifying edge)

edges = [1 2; 2 3; 3 4; 4 1; 1 3];  % Rigid graph

d_star = zeros(4);
d_star(1,2) = d12; d_star(2,1) = d12;
d_star(2,3) = d23; d_star(3,2) = d23;
d_star(3,4) = d34; d_star(4,3) = d34;
d_star(4,1) = d41; d_star(1,4) = d41;
d_star(1,3) = d13; d_star(3,1) = d13;

kP = 0.1;  % control gain
%% Simulation loop
for t = 1:Nsteps-1
    u = zeros(3,4);
    for i = 1:4
        u(:,i) = controlLaw(i, positions, edges, d_star, kP);
    end

    % Euler integration
    for i = 1:4
        positions(:,i) = positions(:,i) + dt * u(:,i);
        traj(:,t+1,i) = positions(:,i);
    end
end

%% Plot 3D trajectories

figure; 
hold on;
colors = {'r', 'g', 'b', 'm'};
labels = {'Drone 1', 'Drone 2', 'Drone 3', 'Drone 4'};

for i = 1:4
    
    hL(i) = plot3(traj(1,:,i), traj(2,:,i), traj(3,:,i), ...
        'Color', colors{i}, 'LineWidth', 1.5);
    
    % Initial Positions
    scatter3(traj(1,1,i), traj(2,1,i), traj(3,1,i), ...
        80, 'b', 'o', 'LineWidth', 1.5, 'MarkerFaceColor', 'none', ...
        'HandleVisibility', 'off');

    % Final positions
    scatter3(traj(1,end,i), traj(2,end,i), traj(3,end,i), ...
        80, 'r', 'x', 'LineWidth', 1.5, ...
        'HandleVisibility', 'off');
end

hStart = plot3(nan, nan, nan, 'ob', 'LineWidth', 1.5);
hEnd   = plot3(nan, nan, nan, 'xr', 'LineWidth', 1.5);

% Legend
legend([hL, hStart, hEnd], ...
       [labels, {'Start Position', 'End Position'}]);


grid on; axis equal;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Square Formation with 4 Drones');
view(3);
%% Plot interaction graph
G = graph(edges(:,1), edges(:,2));
nodeX = [0, s, s, 0];  % ideal square layout
nodeY = [0, 0, s, s];

figure;
plot(G, 'XData', nodeX, 'YData', nodeY, ...
     'NodeColor', 'k', 'MarkerSize', 12, ...
     'NodeLabelColor', 'w', 'LineWidth', 2);
axis equal;
title('Interaction Graph of 4 Drones (Rigid Square)');
