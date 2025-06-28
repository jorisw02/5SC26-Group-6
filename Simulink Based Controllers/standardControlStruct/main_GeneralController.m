clear all;close all;clc

%%
addpath('./_controllers/')
addpath('./_funcs')
addpath('./_models')

%% Some values as also used in the simulation
%  !!Do not change these values without also changing them in simulink!!
sys_param2 = load("sys_param2.mat");

%% Load/Make linearized model
[linSys,G,G_all] = makeLinModel(sys_param2.m,sys_param2.I);
[linSys_unc,G_unc,G_all_unc] = makeUncLinModel(sys_param2.m,sys_param2.I);

%% Time
% Tend = 0.5;  % Total time to run the simulation [s]
Tend = 24.5;  % Total time to run the simulation [s]
% Tend = 5*60;  % Total time to run the simulation [s]
% Tend = 240;  % Total time to run the simulation [s]
Ts = 0.0001; % Sampling time of the simulation  [s]
fs = 1/Ts;  % Sampling frequency of the simulation  [1/s]

tVec = 0:Ts:Tend;   % Time vector used for reference generation [s]

%% Define setpoint
% -1: Step on z
% 0: Hover at starting position
% 1: Ramp with saturation in x, y and z
% 2: Ramp with saturation in z and circle in x and y
% 3: Circle in x, y and z
% 4: Circle in x and y nothing in z
% 5: Ramp in z only
% 6: Figure 8 tilted in Rx
% 7: Figure 8 tilted in Ry
[sim_ref_x,sim_ref_y,sim_ref_z,sim_ref_yaw] = makeSetpoint(6,tVec);

%% Possibility to change controller parameters
if ~exist('./_controllers/standardPID.mat','file')
    makeStandardPID() % Only has to run once to generate the standard controller
end
if ~exist('./_controllers/Hinf_W1.mat','file')
    disp('Making a new Hinf controller')
    makeHinf(linSys,true) % Only has to run once to generate the standard controller
end
if ~exist('./_controllers/musyn_W1.mat','file')
    load('Hinf_W1.mat')
    makeMusyn(linSys_unc,true,K) % Only has to run once to generate the standard controller
end

% Load in a controller
load('standardPID.mat')
load('Hinf_W1.mat')
gamma

load('musyn_W1.mat')
CLtest = lft(linSys_unc,K);
[N,~,~] = lftdata(CLtest);
[STABMARG,WCU] = robstab(CLtest);
   
%% Run the simulation
clc
noiseParam  = makeNoise_V2(true,tVec);

% options for sysID: 'all', 'F', 'Mx', 'My', 'Mz'
% if addSysID(false,...) --> all DOF will be returned
sysID  = addSysID(false,fs,Tend,tVec,'all');
% sysID  = addSysID(true,fs,Tend,tVec,'all');

% Simulation options: 'casPID', 'Hinf', 'SS'
% load('standardPID.mat')
% simOut_PID  = simulateModel_V2('casPID');
% load('Hinf_W1.mat')
% simOut_Hinf = simulateModel_V2('Hinf');
load('musyn_W1.mat')
simOut_SS   = simulateModel_V2('SS');

%% Visualization
makeGeneralPlots(simOut_PID,0)
makeGeneralPlots(simOut_Hinf,10)
makeGeneralPlots(simOut_SS,20)

%% Save figures for the report
baseFig=0;
folderName = 'casPID';
saveFigures_V2

baseFig=10;
folderName = 'Hinf';
saveFigures_V2

baseFig=20;
folderName = 'SS';
saveFigures_V2

%% Weighting filters figures
Wu_Hinf = load('.\_weights\Wfilters1.mat', 'Wu_tot');
Wy_Hinf = load('.\_weights\Wfilters1.mat', 'Wy_tot');

Wu_musyn = load('.\_weights\unc_Wfilters1.mat', 'Wu_tot');
Wy_musyn = load('.\_weights\unc_Wfilters1.mat', 'Wy_tot');

names = Wy_musyn.Wy_tot.outputName;
names{7} = 'phi_{tilde}';
names{8} = 'theta_{tilde}';
names{9} = 'psi_{tilde}';
Wy_musyn.Wy_tot.outputName = names;
Wy_Hinf.Wy_tot.outputName = names;

figure(1002);clf
    bodemag(Wy_Hinf.Wy_tot(7:9,7:9),Wy_musyn.Wy_tot(7:9,7:9));grid minor
        title('Output filters Wy comparison')
        legend('Hinf controller','Robust controller','location','northeast')

%% The data needed for identification
data = iddata(simOut_PID.pos.all,simOut_PID.plantInput.all,Ts);

ID_sys = ssest(data,12);

G_all.InputName = G.InputName;
G_all.OutputName = {'x','y','z','x_{dot}','y_{dot}','z_{dot}','phi','theta','psi','phi_{dot}','theta_{dot}','psi_{dot}'};

figure(2001);clf
    bodemag(testVal([1:3 7:9],:),G_all([1:3 7:9],:));grid minor
        legend('Identified system','Linearized system')
    title('Bode diagram of identified system using "ssest()"')

%% mu-synthesis non-nominal parameters
% To run this code, slight changes to the simulink model need to be made
load('musyn_W1.mat')


m_unc = ureal('m',sys_param2.m,'Percentage',5);

Ixx = ureal('Ixx',sys_param2.I(1,1),'Percentage',5);
Iyy = ureal('Iyy',sys_param2.I(2,2),'Percentage',5);
Izz = ureal('Izz',sys_param2.I(3,3),'Percentage',5);

Ixy = ureal('Ixy',0,'PlusMinus',1e-6);
Ixz = ureal('Ixz',0,'PlusMinus',1e-6);
Iyz = ureal('Iyz',0,'PlusMinus',1e-6);

I_unc = [Ixx,Ixy,Ixz;Ixy,Iyy,Iyz;Ixz,Iyz,Izz];

for i=1:15
    I_unc_sample = usample(I_unc);
    m_unc_sample = usample(m_unc);
    simOut_musyn(i) = simulateModel_V2('SS');
end

%% mu-synthesis non-nominal parameters plottig


figure(3001);clf;
for i=1:15
    plot3(simOut_musyn(i).pos.x(1),simOut_musyn(i).pos.y(1),simOut_musyn(i).pos.z(1),'bo','LineWidth',1.5);hold on;grid minor
    plot3(simOut_musyn(i).pos.x(end),simOut_musyn(i).pos.y(end),simOut_musyn(i).pos.z(end),'rx','LineWidth',1.5)
    plot3(simOut_musyn(i).pos.x,simOut_musyn(i).pos.y,simOut_musyn(i).pos.z)
    try
        xlim([min(min(simOut_musyn(i).pos.x),min(simOut_musyn(i).pos.y)) max(max(simOut_musyn(i).pos.x),max(simOut_musyn(i).pos.y))])
        ylim([min(min(simOut_musyn(i).pos.x),min(simOut_musyn(i).pos.y)) max(max(simOut_musyn(i).pos.x),max(simOut_musyn(i).pos.y))])   
    catch
        xlim([-1 1])
        ylim([-1 1])
    end
    xlabel('x-position [m]')
    ylabel('y-position [m]')
    zlabel('z-position [m]')
    title(['Drone trajectory in 3D, over multiple non-nominal parameters'])
    legend('Start position','End position','location','best')
end
print(['./Figures/4_control/','SS','/nonNom_3Dplot_1'],'-depsc')
        

figure(3002);clf
for i=1:15
    sgtitle('Reference tracking performance for non-nominal parameters')
    subplot(311);hold on
        plot(simOut_musyn(i).tVec,simOut_musyn(i).pos.x);hold on;grid minor
        plot(simOut_musyn(i).tVec,simOut_musyn(i).ref.x,'r--','LineWidth',1.5);
            xlabel('Time [s]')
            ylabel('x-position [m]')
            title(['x-reference tracking'])
            legend('Reference','location','best')
    subplot(312)
        plot(simOut_musyn(i).tVec,simOut_musyn(i).pos.y);hold on;grid minor
        plot(simOut_musyn(i).tVec,simOut_musyn(i).ref.y,'r--','LineWidth',1.5);
            xlabel('Time [s]')
            ylabel('y-position [m]')
            title(['y-reference tracking'])
            legend('Reference','location','best')
    subplot(313)
        plot(simOut_musyn(i).tVec,simOut_musyn(i).pos.z);hold on;grid minor
        plot(simOut_musyn(i).tVec,simOut_musyn(i).ref.z,'r--','LineWidth',1.5);
            xlabel('Time [s]')
            ylabel('z-position [m]')
            title(['z-reference tracking'])
            legend('Reference','location','best')
end
print(['./Figures/4_control/','SS','/nonNom_2Dplot_1'],'-depsc')






