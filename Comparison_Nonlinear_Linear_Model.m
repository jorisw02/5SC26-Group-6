clear; clc; close all;
g = 9.81; 
m = 0.036;
I = diag([1.7e-5, 1.7e-5, 2.9e-5]);
inv_I = inv(I);

%% Initialization 
t = 0:0.01:5;
x_0 = [0; 0; 3; 
       0; 0; 0;
       0; 0; 0;
       0; 0; 0];

% Equilibria
x_eq = zeros(12,1);
psi_eq = x_eq(9);

u_eq  = [m*g;0;0;0];

% State, input, output dimensions
nx = 12;
nu = 4;
ny = 6;

% State matrix
A = zeros(nx,nx);
A(1:3,4:6) = eye(3); 
A(7:9,10:12) = eye(3);
A(4:5, 7:8) = [g*sin(psi_eq), g*cos(psi_eq); -g*cos(psi_eq), g*sin(psi_eq)]; 

% Input matrix
B = zeros(nx,nu);
B(6,1) = 1/m; B(10:12, 2:4) = inv_I;

% Output matrix
C = zeros(ny,nx);
C(1:3,1:3) = eye(3); C(4:6, 7:9) = eye(3);

% Direct Feedthrough matrix
D = zeros(ny, nu);

% Inputs u
f = @(cmd) 2.130295*10^-11*cmd.^2 + 1.032633*10^-6*cmd + 5.484560*10^-4; 
h = @(cmd) 4*f(cmd) - 0.036*9.81;
cmd_hover = fzero(h, [0, 1e5]);

u1_fall = 0;
u1_hover = 4*f(cmd_hover);
u1_up = 4*f(2^16-1); 

u1 = [u1_fall, u1_hover, u1_up];
titles = {
    'Trajectory of quadcopter for no thrust input', 
    'Trajectory of quadcopter for hover thrust input', 
    'Trajectory of quadcopter maximal thrust input'
};
for j = 1:length(u1)
    u = [u1(j); 0;0;0];
    
    options = odeset('Events', @ground_event);
    
    [t_out_lin, x_out_lin] = ode45(@quadcopter_linear_dynamics, t, x_0, options, u, A, B, u_eq);
    y_lin = C*x_out_lin';

    [t_out_nonlin, x_out_nonlin] = ode45(@quadcopter_nonlinear_dynamics, t, x_0, options, u, m, g, I, inv_I);
    y_nonlin = [x_out_nonlin(:,1)';
                x_out_nonlin(:,2)';
                x_out_nonlin(:,3)'];
    
    figure;
    plot3(y_nonlin(1,:), y_nonlin(2,:), y_nonlin(3,:),'LineWidth',1.5); hold on;grid minor
    plot3(y_lin(1,:), y_lin(2,:), y_lin(3,:), 'LineWidth', 1.5);
    plot3(y_nonlin(1,1), y_nonlin(2,1), y_nonlin(3,1),'o','LineWidth',1.5);
    plot3(y_nonlin(1,end), y_nonlin(2,end), y_nonlin(3,end),'x','LineWidth',1.5)
    title(titles(j))
    xlabel('position x')
    ylabel('position y')
    zlabel('position z')
    legend('Nonlinear model', 'Linear model', 'Initial position', 'Final position')
end

function dx = quadcopter_linear_dynamics(t, x, u, A, B, u_eq)
    dx = A*x+B*(u-u_eq);
end

function [value, isterminal, direction] = ground_event(t, x, u, m, g, A, B)
    value = x(3);          % altitude (z position)
    isterminal = 1;        % stop the integration
    direction = -1;        % only detect when x(3) is decreasing to 0
end

function R = rotation_matrix(phi, theta, psi)
    R_x = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    R_y = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    R_z = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];

    R = R_z * R_y * R_x;
end

function dx = quadcopter_nonlinear_dynamics(t, x, u, m, g, I, inv_I)
    phi = x(7); theta = x(8); psi = x(9);
    p = x(10); q = x(11); r = x(12);

    R = rotation_matrix(phi, theta, psi);

    dx(1:3,1) = [x(4); x(5); x(6)];
    dx(4:6,1) = 1/m*([0;0;-m*g]+R*[0;0;u(1)]);    

    dx(7:9,1) = [1, 0,        -sin(theta); 
                 0, cos(phi),  sin(phi)*cos(theta);
                 0, -sin(phi), cos(phi)*cos(theta)]* [p;q;r];
    
    cross_term = cross([p;q;r], I*[p;q;r]); 

    dx(10:12,1) = inv_I* ([u(2); u(3); u(4)] - cross_term);
end
