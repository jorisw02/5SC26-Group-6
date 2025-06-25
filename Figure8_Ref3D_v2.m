function [t_vec, x_vec, y_vec, z_vec] = Figure8_Ref3D_v2(TotalTime, Ts, zAmplitude, tilt_direction, N)
%% Generate the 3D lemniscate (figure-8) reference waypoints to fly the drone
% TotalTime: The time it takes to complete the trajectory
% Ts: Sampling time
% zAmplitude: maximum height of the drone plus 1 meter
% tilt_direction: 
% - 'pitched', varying z in the y-direction
% - 'rolled', varying z in the x-direction
% N: Receding Horizon MPC
TotalTime = TotalTime - 4.5;
% Define parameters for normalized lemniscate shape
t = linspace(0,2*pi,101); % Parameter range for full loop

% 3D figure-8 shape
x = sin(t);                        
y = sin(t).* cos(t);              
if strcmp(tilt_direction, 'pitched')
    z = zAmplitude * sin(2*t) + 1;     
elseif strcmp(tilt_direction, 'rolled')
    z = zAmplitude * sin(t) + 1;
end

% Combine waypoints
waypoints = [x; y; z];
timeSteps = round(TotalTime / Ts) + 1;

% Normalized timepoints for trajectory generation
normalized_timepoints = linspace(0, 1, size(waypoints, 2));

% Generate smooth minimum-jerk trajectory
[q, ~, ~, ~, ~, ~, tvec_normalized] = minjerkpolytraj(waypoints, normalized_timepoints, timeSteps);

% Scale time back to match desired TotalTime
tvec = tvec_normalized * TotalTime;
tvec = tvec + 2;

% Get position vectors
x_vec = q(1,:)';
y_vec = q(2,:)';
z_vec = q(3,:)';

% Add 1-second ascent at beginning
N_hover1 = round(1 / Ts); % 1 second of hovering

t_up = 0:Ts:1-Ts; t_up = t_up';
x_up = zeros(N_hover1, 1);
y_up = zeros(N_hover1,1);
z_up = linspace(0.01, 1, N_hover1)';

% Add 1-second hover at beginning
t_hover1 = 1:Ts:2-Ts;
x_hover1 = zeros(N_hover1,1);
y_hover1 = zeros(N_hover1,1);
z_hover1 = ones(N_hover1,1);

% Add 1-second hover at final position
x_last = x_vec(end);
y_last = y_vec(end);
z_last = z_vec(end);

t_hover2 = tvec(end) + Ts * (1:N_hover1)';
x_hover2 = x_last * ones(N_hover1, 1);
y_hover2 = y_last * ones(N_hover1, 1);
z_hover2 = z_last * ones(N_hover1, 1);

% Add 1.5-second descent
N_hover2 = round(0.5 / Ts); % 0.5 seconds hover low
N_down = round(1 / Ts);     % 1.0 second descent
N_total = N_down + N_hover2;

t_flying_down = t_hover2(end) + Ts * (1:N_total)';

z_down = linspace(1, 0.05, N_down)';
z_hover_low = 0.05 * ones(N_hover2, 1);

x_down = zeros(N_total, 1); % Stay in place
y_down = zeros(N_total, 1);

%% Combine everything
t_vec = [t_up; t_hover1'; tvec'; t_hover2; t_flying_down];
x_vec = [x_up; x_hover1; x_vec; x_hover2; x_down; zeros(N,1)];
y_vec = [y_up; y_hover1; y_vec; y_hover2; y_down; zeros(N,1)];
z_vec = [z_up; z_hover1; z_vec; z_hover2; z_down; z_hover_low; 0.05*ones(N,1)];

% Transpose for output
t_vec = t_vec';
x_vec = x_vec';
y_vec = y_vec';
z_vec = z_vec';

% Visualisation
% figure;
% plot3(x_vec, y_vec, z_vec, 'LineWidth', 1.5);
% xlabel('X'); ylabel('Y'); zlabel('Z');
% grid on; axis equal;
% title('3D Figure-8 Tracking trajectory');
% grid minor;
% 
% figure;
% t = tiledlayout(3,1); % 3 rows, 1 column
% t.TileSpacing = 'compact'; % or 'loose' / 'none'
% t.Padding = 'compact';     % reduce outer padding
% sgtitle('Tracking trajectory in each direction')
% 
% nexttile
% plot(t_vec, x_vec(1:end-N), 'LineWidth', 1.5); 
% xlabel('Time (s)'); ylabel('X position'); 
% grid on; axis equal;
% title('Figure-8 tracking trajectory in x-direction');
% xlim([0, TotalTime+4.5])
% ylim([-1, 1])
% grid minor;
% 
% nexttile
% plot(t_vec, y_vec(1:end-N), 'LineWidth', 1.5); 
% xlabel('Time (s)'); ylabel('Y position'); 
% grid on; axis equal;
% title('Figure-8 tracking trajectory in y-direction');
% xlim([0, TotalTime+4.5])
% ylim([-1, 1])
% grid minor;
% 
% nexttile
% plot(t_vec, z_vec(1:end-N), 'LineWidth', 1.5); 
% xlabel('Time (s)'); ylabel('Z position'); 
% grid on; axis equal;
% title('Figure-8 tracking trajectory in z-direction');
% xlim([0, TotalTime+4.5])
% ylim([0, 2])
% grid minor;
end
