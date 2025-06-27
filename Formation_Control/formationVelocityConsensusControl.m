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

% Desired positions for drones in formation
x_1 = 0.5; y_1 = 0.5; z_1 = 1.5;
s = 0.5; % displacement in x and y for formation
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
      1+2*eps];
P2 = [x_1-s - eps;
      y_1-s - eps;
      1-eps];
P3 = [x_1-s - 1.5*eps;
      y_1+s - 1.5*eps;
      1+eps];
V_ic = [0.01;0.01;0.01];
IC = [P1;P2;P3;V_ic;V_ic;V_ic];

% time span of simulation
tspan = [0 0.05];
final = [];

for i = 1:10000
    [t, p] = ode45(@(t, x) formation_velocity_consensus(t,x,L,P_desired),tspan,IC);
    if i > 1000
        V_ic(1) = p(end,10)*cos(0.2*t(end)*(i-1000));
        V_ic(2) = p(end,11)*sin(0.2*t(end)*(i-1000));
    end
    IC = [p(end,1:9).';V_ic;V_ic;V_ic];
    final = [final;p(end,:)];
end

%% Plotting
figure(1)
plot3(final(:,1), final(:,2), final(:,3),'Color','b', Linewidth=2)
hold on
grid on
plot3(final(:,1+3), final(:,2+3), final(:,3+3),'Color','r', Linewidth=2)
plot3(final(:,1+6), final(:,2+6), final(:,3+6),'Color','g', Linewidth=2)
legend('p1','p2','p3')
% marking initial positions
% plot3(P1(1), P1(2), P1(3),'X',MarkerSize=10)
% plot3(P2(1), P2(2), P2(3),'X',MarkerSize=10)
% plot3(P3(1), P3(2), P3(3),'X',MarkerSize=10)
