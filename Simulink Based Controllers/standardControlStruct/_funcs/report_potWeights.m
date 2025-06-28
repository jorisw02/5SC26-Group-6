load('C:\Users\maxja\Documents\(4)School\Master\Q8\5SC26_IntegrationProject\Work\General\standardControlStruct\_weights\Wfilters1.mat')


%%
Ys = {'x','y','z','phi','theta','psi'};

freqVec = logspace(-4,6,1000);
figure(501);clf
set(gcf,'Position',[100 80 1100 900])
    for i=1:19
        subplot(5,4,i)
            [mag,phs,f] = bode(Wy_tot(i,i),freqVec);
            semilogx(f,db(squeeze(mag)));grid minor
            xlim([freqVec(1) freqVec(end)])
                xlabel('Frequency [Hz]')
                ylabel('Magnitude [dB]')
                if i<=13
                    title(['Output W-filter on ',Wy_tot(i,i).InputName])
                else
                    title(['Output W-filter on control channel ',Ys(i-13)])
                end
    end
print('./Figures/4_control/Bodemags_Wy_all_1','-depsc')

%%
Ys = {'x','y','z','phi','theta','psi'};

freqVec = logspace(-4,6,1000);
figure(502);clf
set(gcf,'Position',[100 80 1100 550])
    for i=1:11
        subplot(3,4,i)
            [mag,phs,f] = bode(Wu_tot(i,i),freqVec);
            semilogx(f,db(squeeze(mag)));grid minor
            xlim([freqVec(1) freqVec(end)])
                xlabel('Frequency [Hz]')
                ylabel('Magnitude [dB]')
                if i<=13
                    title(['Output W-filter on ',Wy_tot(i,i).InputName])
                else
                    title(['Output W-filter on control channel ',Ys(i-13)])
                end
    end
print('./Figures/4_control/Bodemags_Wu_all_1','-depsc')








