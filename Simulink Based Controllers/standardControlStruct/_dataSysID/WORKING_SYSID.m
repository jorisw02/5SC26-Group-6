% Parameters
Fs = 10000; % Sampling frequency in Hz
% NFFT = 1024; % FFT length
NFFT = 102400*5; % FFT length
% NFFT = 245010/2; % FFT length

% Calculate window and overlap
window = hanning(NFFT);
noverlap = round(NFFT * 0.5);

% Input and output signals (replace with your data)
% input_signal_z = simOut_Hinf.plantInput.F(1:Tend/Ts/10)';  % Example force signal
% output_signal_z = mean(reshape(simOut_Hinf.pos.z(Tend/Ts/10*6:Tend/Ts-1)',[],4),2);  % Example response signal

input_signal_z = simOut_Hinf.plantInput.F';  % Example force signal
output_signal_z = simOut_Hinf.pos.z;  % Example response signal
input_signal_phi = simOut_Hinf.plantInput.M1';  % Example force signal
output_signal_phi = simOut_Hinf.pos.phi;  % Example response signal
input_signal_theta = simOut_Hinf.plantInput.M2';  % Example force signal
output_signal_theta = simOut_Hinf.pos.theta;  % Example response signal
input_signal_psi = simOut_Hinf.plantInput.M3';  % Example force signal
output_signal_psi = simOut_Hinf.pos.psi;  % Example response signal

% Compute FRF using H1 estimator: H1 = Gxy / Gxx
[pxx_z, f_z] = cpsd(input_signal_z, input_signal_z, window, noverlap, NFFT, Fs); % Gxx
[pxy_z, ~] = cpsd(output_signal_z, input_signal_z, window, noverlap, NFFT, Fs); % Gxy
[pyy_z, ~] = cpsd(output_signal_z, output_signal_z, window, noverlap, NFFT, Fs); % Gxy
H_z = pxy_z ./ pxx_z;
IDed_z = frd((H_z),f_z*(2*pi));
coh_z = (abs(pxy_z).^2) ./ (pxx_z .* pyy_z);

% Compute FRF using H1 estimator: H1 = Gxy / Gxx
[pxx_phi, f_phi] = cpsd(input_signal_phi, input_signal_phi, window, noverlap, NFFT, Fs); % Gxx
[pxy_phi, ~] = cpsd(output_signal_phi, input_signal_phi, window, noverlap, NFFT, Fs); % Gxy
[pyy_phi, ~] = cpsd(output_signal_phi, output_signal_phi, window, noverlap, NFFT, Fs); % Gxy
H_phi = pxy_phi ./ pxx_phi;
IDed_phi = frd((H_phi),f_phi*(2*pi));
coh_phi = (abs(pxy_phi).^2) ./ (pxx_phi .* pyy_phi);

% Compute FRF using H1 estimator: H1 = Gxy / Gxx
[pxx_theta, f_theta] = cpsd(input_signal_theta, input_signal_theta, window, noverlap, NFFT, Fs); % Gxx
[pxy_theta, ~] = cpsd(output_signal_theta, input_signal_theta, window, noverlap, NFFT, Fs); % Gxy
[pyy_theta, ~] = cpsd(output_signal_theta, output_signal_theta, window, noverlap, NFFT, Fs); % Gxy
H_theta = pxy_theta ./ pxx_theta;
IDed_theta = frd((H_theta),f_theta*(2*pi));
coh_theta = (abs(pxy_theta).^2) ./ (pxx_theta .* pyy_theta);

% Compute FRF using H1 estimator: H1 = Gxy / Gxx
[pxx_psi, f_psi] = cpsd(input_signal_psi, input_signal_psi, window, noverlap, NFFT, Fs); % Gxx
[pxy_psi, ~] = cpsd(output_signal_psi, input_signal_psi, window, noverlap, NFFT, Fs); % Gxy
[pyy_psi, ~] = cpsd(output_signal_psi, output_signal_psi, window, noverlap, NFFT, Fs); % Gxy
[coh_psi2, ~] = mscohere(input_signal_psi, output_signal_psi, window, noverlap, NFFT, Fs); % Gxy
H_psi = pxy_psi ./ pxx_psi;
IDed_psi = frd((H_psi),f_psi*(2*pi));
coh_psi = (abs(pxy_psi).^2) ./ (pxx_psi .* pyy_psi);

%%
mFreq = 4;

figure(1001);clf
set(gcf,'Position',[100 100 1000 500])
    subplot(221)
        bodemag(G(3,1),IDed_z);grid minor
        xline(mFreq,'m-.','linewidth',1)
            title('Transfer from u_1 to z')
            legend('Linearized model','Identified model','4rad/s line','location','best')
    subplot(222)
        bodemag(G(4,2),IDed_phi);grid minor
        xline(mFreq,'m-.','linewidth',1)
            title('Transfer from u_2 to phi')
            legend('Linearized model','Identified model','4rad/s line','location','best')
    subplot(223)
        bodemag(G(5,3),IDed_theta);grid minor
        xline(mFreq,'m-.','linewidth',1)
            title('Transfer from u_3 to theta')
            legend('Linearized model','Identified model','4rad/s line','location','best')
    subplot(224)
        bodemag(G(6,4),IDed_psi);grid minor
        xline(mFreq,'m-.','linewidth',1)
            title('Transfer from u_4 to psi')
            legend('Linearized model','Identified model','4rad/s line','location','best')
print('./Figures/3_sysID/Bodemags_SYSID_1','-depsc')

%%
figure(1002);clf
set(gcf,'Position',[100 100 1000 500])
    subplot(221)
        semilogx(f_z,coh_z);grid minor
        xlabel('Frequency (rad/s)')
        ylabel('Coherence (-)')
            title('Coherence of identified transfer from u_1 to z')
    subplot(222)
        semilogx(f_phi,coh_phi);grid minor
        xlabel('Frequency (rad/s)')
        ylabel('Coherence (-)')
            title('Coherence of identified transfer from u_2 to phi')
    subplot(223)
        semilogx(f_theta,coh_theta);grid minor
        xlabel('Frequency (rad/s)')
        ylabel('Coherence (-)')
            title('Coherence of identified transfer from u_3 to theta')
    subplot(224)
        semilogx(f_psi,coh_psi);grid minor;hold on
        semilogx(f_psi,coh_psi2);
        xlabel('Frequency (rad/s)')
        ylabel('Coherence (-)')
            title('Coherence of identified transfer from u_4 to psi')
print('./Figures/3_sysID/Coherence_SYSID_1','-depsc')

%%
clc
m_id = abs(freqresp(IDed_z,mFreq))
m_real = G.B(6,1) %abs(freqresp(G(3,1),mFreq)) %
% m_id/m_real

Ixx_id = abs(freqresp(IDed_phi,mFreq));
Ixx_real = abs(freqresp(G(4,2),mFreq));%G.B(10,2);
% Ixx_id/Ixx_real

Iyy_id = abs(freqresp(IDed_theta,mFreq))/180*pi;
Iyy_real = G.B(11,3);
% Iyy_id/Iyy_real

Izz_id = abs(freqresp(IDed_psi,mFreq))/180*pi;
Izz_real = G.B(12,4);
% Izz_id/Izz_real



%%

save('./_dataSysID/ID_all.mat','simOut_Hinf','IDed_z','IDed_phi','IDed_theta','IDed_psi',...
    'coh_z','f_z','coh_phi','f_phi','coh_theta','f_theta','coh_psi','f_psi',...
    'window','noverlap','NFFT','Fs','sysID');
















