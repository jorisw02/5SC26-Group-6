

Ts = 0.01;
%% Load setpoint Figure O
addpath('D:\BagFiles\Figure8')
[SP,tVec_SP] = SPBuilding(Ts);

%% Import Vicon data of Figure O
addpath('D:\BagFiles\Figure8');
test3July = importfile('/test_3July');
tVec_VICON = test3July.VarName1/100;

wndw_VICON = (1325:2245);

% figure(2);clf;
%     subplot(311);hold on
%         plot(tVec_VICON(wndw_VICON)-tVec_VICON(wndw_VICON(1)),test3July.VarName6(wndw_VICON)./1000)
%         plot(tVec_SP,SP.x)
%     subplot(312);hold on
%         plot(tVec_VICON(wndw_VICON)-tVec_VICON(wndw_VICON(1)),test3July.VarName7(wndw_VICON)./1000)
%         plot(tVec_SP,SP.y)
%     subplot(313);hold on
%         plot(tVec_VICON(wndw_VICON)-tVec_VICON(wndw_VICON(1)),test3July.VarName8(wndw_VICON)./1000)
%         plot(tVec_SP,SP.z)

%% Run Sim with Figure O
Tend = 14.5;  % Total time to run the simulation [s]
Ts = 0.0001; % Sampling time of the simulation  [s]
fs = 1/Ts;  % Sampling frequency of the simulation  [1/s]

tVec = 0:Ts:Tend;   % Time vector used for reference generation [s]

waitTime = 2;
SP.x_longer = [zeros(1,waitTime/Ts) SP.x];
SP.y_longer = [zeros(1,waitTime/Ts) SP.y];
SP.z_longer = [SP.z ones(1,waitTime/Ts)];

tVec_SP_longer = [0:Ts:waitTime-Ts tVec_SP+2-Ts];

[~,~,~,sim_ref_yaw] = makeSetpoint(0,tVec);
sim_ref_x  = timetable(seconds(tVec_SP_longer'),SP.x_longer');
sim_ref_y  = timetable(seconds(tVec_SP_longer'),SP.y_longer');
sim_ref_z  = timetable(seconds(tVec_SP_longer'),SP.z_longer');

load('standardPID.mat')
   
noiseParam  = makeNoise_V2(true,tVec);
sysID  = addSysID(false,fs,Tend,tVec,'all');

load('standardPID.mat')
simOut_PID = simulateModel_V2('casPID');

%%
wndw_PID = 1300:5900;

figure(1);clf;
    set(gcf,'Position',[100 100 750 450])
    subplot(311);hold on;grid minor
        plot(tVec_VICON(wndw_VICON)-tVec_VICON(wndw_VICON(1)),test3July.VarName6(wndw_VICON)./1000,'LineWidth',2)
        plot(simOut_PID.tVec(wndw_PID)-simOut_PID.tVec(wndw_PID(1)),simOut_PID.pos.x(wndw_PID),'LineWidth',2)
        plot(tVec_SP,SP.x,'--','LineWidth',2)
        xlim([0 9]);
            xlabel('Time [s]')
            ylabel('Position [m]')
                rms_xe_sim = rmse(downsample(simOut_PID.pos.x(wndw_PID),5)',SP.x(1:921));
                rms_xe_msr = rmse(test3July.VarName6(wndw_VICON)./1000,SP.x(1:921)');
            title(['x-reference tracking, RMSE simulation:',num2str(rms_xe_sim),', RMSE measurement:',num2str(rms_xe_msr)])
            legend('VICON measurement','Simulink simulation','Setpoint','location','best')
    subplot(312);hold on;grid minor
        plot(tVec_VICON(wndw_VICON)-tVec_VICON(wndw_VICON(1)),test3July.VarName7(wndw_VICON)./1000,'LineWidth',2)
        plot(simOut_PID.tVec(wndw_PID)-simOut_PID.tVec(wndw_PID(1)),simOut_PID.pos.y(wndw_PID),'LineWidth',2)
        plot(tVec_SP,SP.y,'--','LineWidth',2)
        xlim([0 9]);
            xlabel('Time [s]')
            ylabel('Position [m]')
                rms_ye_sim = rmse(downsample(simOut_PID.pos.y(wndw_PID),5)',SP.y(1:921));
                rms_ye_msr = rmse(test3July.VarName7(wndw_VICON)./1000,SP.y(1:921)');
            title(['y-reference tracking, RMSE simulation:',num2str(rms_ye_sim),', RMSE measurement:',num2str(rms_ye_msr)])
            legend('VICON measurement','Simulink simulation','Setpoint','location','best')
    subplot(313);hold on;grid minor
        plot(tVec_VICON(wndw_VICON)-tVec_VICON(wndw_VICON(1)),test3July.VarName8(wndw_VICON)./1000,'LineWidth',2)
        plot(simOut_PID.tVec(wndw_PID)-simOut_PID.tVec(wndw_PID(1)),simOut_PID.pos.z(wndw_PID),'LineWidth',2)
        plot(tVec_SP,SP.z,'--','LineWidth',2)
        xlim([0 9]);
        ylim([0.98 1.1]);
            xlabel('Time [s]')
            ylabel('Position [m]')
                rms_ze_sim = rmse(downsample(simOut_PID.pos.z(wndw_PID),5)',SP.z(1:921));
                rms_ze_msr = rmse(test3July.VarName8(wndw_VICON)./1000,SP.z(1:921)');
            title(['z-reference tracking, RMSE simulation:',num2str(rms_ze_sim),', RMSE measurement:',num2str(rms_ze_msr)])
            legend('VICON measurement','Simulink simulation','Setpoint','location','best')

    print('./Figures/3_sysID/modelValidation_2D_1','-depsc')

%%
figure(2);clf;
    set(gcf,'Position',[100 100 750 250])
    subplot(311);hold on;grid minor
        plot(tVec_VICON(wndw_VICON)-tVec_VICON(wndw_VICON(1)),test3July.VarName6(wndw_VICON)'./1000-SP.x(1:921))
        plot(downsample(simOut_PID.tVec(wndw_PID),5)-simOut_PID.tVec(wndw_PID(1)),downsample(simOut_PID.pos.x(wndw_PID),5)'-SP.x(1:921))
            xlabel('Time [s]')
            ylabel('Error [m]')
            title('x-position [m]')
            legend('VICON error','Simulation error','location','best')
    subplot(312);hold on;grid minor
        plot(tVec_VICON(wndw_VICON)-tVec_VICON(wndw_VICON(1)),test3July.VarName7(wndw_VICON)'./1000-SP.y(1:921))
        plot(downsample(simOut_PID.tVec(wndw_PID),5)-simOut_PID.tVec(wndw_PID(1)),downsample(simOut_PID.pos.y(wndw_PID),5)'-SP.y(1:921))
            xlabel('Time [s]')
            ylabel('Error [m]')
            title('y-position [m]')
            legend('VICON error','Simulation error','location','best')
    subplot(313);hold on;grid minor
        plot(tVec_VICON(wndw_VICON)-tVec_VICON(wndw_VICON(1)),test3July.VarName8(wndw_VICON)'./1000-SP.z(1:921))
        plot(downsample(simOut_PID.tVec(wndw_PID),5)-simOut_PID.tVec(wndw_PID(1)),downsample(simOut_PID.pos.z(wndw_PID),5)'-SP.z(1:921))
            xlabel('Time [s]')
            ylabel('Error [m]')
            title('z-position [m]')
            legend('VICON error','Simulation error','location','best')
%%
figure(3);clf;
    plot3(test3July.VarName6(wndw_VICON)./1000,test3July.VarName7(wndw_VICON)./1000,test3July.VarName8(wndw_VICON)./1000,'LineWidth',2);hold on;grid minor
    plot3(simOut_PID.pos.x(wndw_PID),simOut_PID.pos.y(wndw_PID),simOut_PID.pos.z(wndw_PID),'LineWidth',2)
    plot3(SP.x,SP.y,SP.z,'-.','LineWidth',2)
    zlim([0 1.5])
        xlabel('x-position [m]')
        ylabel('y-position [m]')
        zlabel('z-position [m]')
        e = [simOut_PID.pos.x simOut_PID.pos.y simOut_PID.pos.z]' - [simOut_PID.ref.x simOut_PID.ref.y simOut_PID.ref.z]';
        RMS_pos_e = sqrt(sum(vecnorm(e, 2, 1).^2) / length(simOut_PID.ref.x));
        title(['Drone trajectories with Figure 0, RMS error: ',num2str(RMS_pos_e)])
        legend('VICON measurement','Simulink simulation','Setpoint','location','northeast')

    print('./Figures/3_sysID/modelValidation_3D_1','-depsc')




