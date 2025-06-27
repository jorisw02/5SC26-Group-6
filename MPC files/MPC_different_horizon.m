clear; clc; close all;
addpath('./_functions')
load('sys_param')

%% Initialization 
Ts = 0.01;              
N_horizon = 1:1:30;    

x0 = [0; 0; 0.01;       
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

% Continuous system
sys_c = ss(A, B, C, D); 

% Discrete time system
sys_d = c2d(sys_c, Ts);
Ad = sys_d.A; Bd = sys_d.B; Cd = sys_d.C; Dd = sys_d.D;

% Cost matrices
Q = eye(nx);
R = 1e-8*eye(nu);
Qr = diag([1e0,1e0,1e0,1e-5, 1e-5,1e-2]);
P = Qr;

%% Thrust and Torques of rotors
f = @(cmd) 2.130295*10^-11*cmd.^2 + 1.032633*10^-6*cmd + 5.484560*10^-4; 
t = @(f) 0.005964552*f + 1.563383* 10^-5;

Mx_max = L*f(2^16-1);
Mx_min = -L*f(2^16-1);

My_max = L*f(2^16-1);
My_min = -L*f(2^16-1);

Mz_max = 2*t(f(2^16-1));
Mz_min = -2*t(f(2^16-1));

%% Constraint sets
% Input bounds
umin = [0; Mx_min; My_min; Mz_min];
umax = [4*f(2^16-1); Mx_max; My_max; Mz_max];

% Output bounds
ymin = [-2; -2; 0; -2*pi; -2*pi; -2*pi];
ymax = [2; 2; 3.5; 2*pi; 2*pi; 2*pi];


results = struct();
for idx = 1:length(N_horizon)
    N = N_horizon(idx);
    % input constraints
    Hu = [-eye(nu); eye(nu)];
    hu = [-umin; umax];
    Hu_bar = kron(eye(N),Hu);
    hu_bar = kron(ones(N,1),hu);
    
    % output constraints
    Hy = [-eye(ny); eye(ny)];
    hy = [-ymin; ymax];
    Hy_bar = kron(eye(N), Hy);
    hy_bar = kron(ones(N,1),hy);
    
    % Initializing the matrices for MPC algorithm
    [Phi, Gamma, Omega, Psi] = mpc_obj(Ad, Bd, Cd, Qr, R, P, N);
    
    %% MPC Algorithm
    % Setting up the MPC optimization algorithm
    x_0 = sdpvar(nx, 1);        % x(k) = x_0|k
    
    U = sdpvar(nu*N, 1);        % U_k = u_0|k, ... , u_{N-1}|k
    U_eq = repmat([m*g;0;0;0], N,1);
    
    Y = sdpvar(ny*N, 1);        % Y_k = y_1|k, ... , y_N|k
    Y_0 = sdpvar(ny,1);         % y(k) = y_0|k
    
    Rk = sdpvar(ny*N, 1);       % R_k = r_0|k, r_1|k, ... , r_N|k
    Rk_0 = sdpvar(ny,1);        % r(k) = r_0|k
    
    % Objective function
    Obj = [Y-Rk; U]' * blkdiag(Omega, Psi) * [Y-Rk; U] + ...
        (Y_0 - Rk_0)' * Qr * (Y_0 - Rk_0);
    
    % Constraints
    Con = [Y == Phi*x_0 + Gamma*(U-U_eq)]; 
    Con = [Con, blkdiag(Hu_bar, Hy_bar)*[U;Y]<=[hu_bar;hy_bar]];
    
    % Define options for solver ( optional )
    options = sdpsettings('solver','quadprog');
    
    % Create the MPC controller using " optimizer "
    Param_In = {x_0, Rk, Y_0, Rk_0};
    Param_Out = {U};
    MPC_sparse = optimizer(Con, Obj, options , Param_In , Param_Out);
    
    %% Calling the function
    TotalTime = 25;
    zAmplitude = 0.5; 
    tilt_direction = 'rolled';
    [t_vec, x_vec_ref, y_vec_ref, z_vec_ref] = Figure8_Ref3D_v2(TotalTime, Ts, ...
        zAmplitude, tilt_direction, N);
   
    rvec = [x_vec_ref; y_vec_ref; z_vec_ref; ...
        zeros(1,length(x_vec_ref)); zeros(1,length(y_vec_ref));zeros(1,length(z_vec_ref))];
    
    k_sim = length(t_vec);
    
    x = zeros(nx, k_sim);
    y = zeros(ny, k_sim);
    u = zeros(nu, k_sim-1);  
    
    % Initial conditions
    x(:,1) = x0;
    y(:,1) = [x0(1:3,1); x0(7:9,1)];
    
    for i = 1:k_sim-1
        rvec_temp = reshape(rvec(:,i+1:i+N), [], 1);
        U_opt = MPC_sparse{x(:,i), rvec_temp, y(:,i), rvec(:,i)};      
    
        u(:, i) = U_opt(1:nu); 

        time = [0 Ts];
        [time,x_temp] = ode45(@quadcopter_nonlinear_dynamics, time, x(:,i), [], u(:,i), m, g, I, inv_I);
        
        x(:,i+1) = x_temp(end,:)';
        y(:,i+1) = C*x(:,i+1);
    end
    results(idx).N = N;
    results(idx).y = y;
    results(idx).u = u;
    results(idx).rvec = rvec;
end

%% Error 
RMS_pos_e = zeros(1,  length(N_horizon));

for i = 1:length(N_horizon)
    e = results(i).y(1:3,:) - results(i).rvec(1:3, 1:end-N_horizon(i));
    RMS_pos_e(i) = sqrt(sum(vecnorm(e, 2, 1).^2) / k_sim);
end

%% Plotting error
figure; 
plot(N_horizon, RMS_pos_e);
xlabel('Horizon N','Interpreter','latex')
ylabel('RMS error','Interpreter','latex')
title('RMS error for different horizons','Interpreter','latex')
xlim([0, N_horizon(end)])
grid on
