
function makeGeneralPlots(simOut,baseFig)
    figure(1+baseFig);clf;
        plot3(simOut.pos.x(1),simOut.pos.y(1),simOut.pos.z(1),'o','LineWidth',1.5);hold on;grid minor
        plot3(simOut.pos.x(end),simOut.pos.y(end),simOut.pos.z(end),'x','LineWidth',1.5)
        plot3(simOut.pos.x,simOut.pos.y,simOut.pos.z,'LineWidth',1.5)
            try
                xlim([min(min(simOut.pos.x),min(simOut.pos.y)) max(max(simOut.pos.x),max(simOut.pos.y))])
                ylim([min(min(simOut.pos.x),min(simOut.pos.y)) max(max(simOut.pos.x),max(simOut.pos.y))])   
            catch
                xlim([-1 1])
                ylim([-1 1])
            end
            xlabel('x-position [m]')
            ylabel('y-position [m]')
            zlabel('z-position [m]')
            e = [simOut.pos.x simOut.pos.y simOut.pos.z]' - [simOut.ref.x simOut.ref.y simOut.ref.z]';
            RMS_pos_e = sqrt(sum(vecnorm(e, 2, 1).^2) / length(simOut.ref.x));
            title(['Drone trajectory in 3D, RMS error: ',num2str(RMS_pos_e)])
            legend('Start position','End position','Drone trajectory','location','best')
        
    % forces = makeForces(simOut.controllerOutput);
    figure(2+baseFig);clf;
        sgtitle('Control effort of the drone over time')
        subplot(211)
            plot(simOut.tVec,simOut.controllerOutput.F,'LineWidth',1.5);hold on;grid minor
                xlabel('Time [s]')
                ylabel('Force [cmd]')
                title('Thrust force')
                legend('Thrust','location','best')
        subplot(212)
            plot(simOut.tVec,simOut.controllerOutput.M1,'LineWidth',1.5);hold on;grid minor
            plot(simOut.tVec,simOut.controllerOutput.M2,'LineWidth',1.5);
            plot(simOut.tVec,simOut.controllerOutput.M3,'LineWidth',1.5);
                xlabel('Time [s]')
                ylabel('Moment [cmd]')
                title('Moments')
                legend('M1','M2','M3','location','best')
        
    figure(3+baseFig);clf;
        sgtitle('Reference tracking performance')
        subplot(311)
            plot(simOut.tVec,simOut.pos.x,'LineWidth',1.5);hold on;grid minor
            plot(simOut.tVec,simOut.ref.x,'--','LineWidth',1.5);
            % ylim([0.29 0.31])
                xlabel('Time [s]')
                ylabel('x-position [m]')
                    norm2_xe = norm(simOut.pos.x-simOut.ref.x);
                    rms_xe = rmse(simOut.pos.x,simOut.ref.x);
                title(['x-reference tracking, RMS error in x-direction:',num2str(rms_xe)])
                legend('Drone position','Reference','location','best')
        subplot(312)
            plot(simOut.tVec,simOut.pos.y,'LineWidth',1.5);hold on;grid minor
            plot(simOut.tVec,simOut.ref.y,'--','LineWidth',1.5);
            % ylim([0.29 0.31])
                xlabel('Time [s]')
                ylabel('y-position [m]')
                    norm2_ye = norm(simOut.pos.y-simOut.ref.y);
                    rms_ye = rmse(simOut.pos.y,simOut.ref.y);
                title(['y-reference tracking, RMS error in y-direction:',num2str(rms_ye)])
                legend('Drone position','Reference','location','best')
        subplot(313)
            plot(simOut.tVec,simOut.pos.z,'LineWidth',1.5);hold on;grid minor
            plot(simOut.tVec,simOut.ref.z,'--','LineWidth',1.5);
            % ylim([0.29 0.31])
                xlabel('Time [s]')
                ylabel('z-position [m]')
                    norm2_ze = norm(simOut.pos.z-simOut.ref.z);
                    rms_ze = rmse(simOut.pos.z,simOut.ref.z);
                title(['z-reference tracking, RMS error in z-direction:',num2str(rms_ze)])
                legend('Drone position','Reference','location','best')
        
    figure(4+baseFig);clf;
        sgtitle('Reference tracking performance')
        subplot(311)
            plot(simOut.tVec,simOut.pos.phi,'LineWidth',1.5);hold on;grid minor
            plot(simOut.tVec,zeros(size(simOut.pos.phi)),'--','LineWidth',1.5);
                xlabel('Time [s]')
                ylabel('Roll [deg]')
                title('Roll tracking')
                legend('Drone orientation','Reference','location','best')
        subplot(312)
            plot(simOut.tVec,simOut.pos.theta,'LineWidth',1.5);hold on;grid minor
            plot(simOut.tVec,zeros(size(simOut.pos.theta)),'--','LineWidth',1.5);
                xlabel('Time [s]')
                ylabel('Pitch [deg]')
                title('Pitch tracking')
                legend('Drone orientation','Reference','location','best')
        subplot(313)
            plot(simOut.tVec,simOut.pos.psi,'LineWidth',1.5);hold on;grid minor
            plot(simOut.tVec,zeros(size(simOut.pos.psi)),'--','LineWidth',1.5);
                xlabel('Time [s]')
                ylabel('Yaw [deg]')
                title('Yaw tracking')
                legend('Drone orientation','Reference','location','best')
end
