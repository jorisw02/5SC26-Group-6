for i=1 % Weigthing filters 1
%% Input filters
%% Ref filters

% Reference input
rx_max = 1; %[m]
f_rx = 0.14; %[Hz]
Wr_x = makeweight(rx_max,[f_rx*2*pi,db2mag(-3)*rx_max],0.001*rx_max,0,4);
    Wr_x.InputName = 'ref(1)';
    Wr_x.OutputName = 'ref_tilde(1)';

% Assuming that y has same limitations as x
% Wr_y = Wr_x;
%     Wr_y.InputName = 'ref(2)';
%     Wr_y.OutputName = 'ref_tilde(2)';
ry_max = 0.5; %[m]
f_ry = 0.14; %[Hz]
Wr_y = makeweight(ry_max,[f_ry*2*pi,db2mag(-3)*ry_max],0.001*ry_max,0,4);
    Wr_y.InputName = 'ref(2)';
    Wr_y.OutputName = 'ref_tilde(2)';

rz_max = 1.5; %[m]
f_rz = 0.14; %[Hz]
Wr_z = makeweight(rz_max,[f_rz*2*pi,db2mag(-3)*rz_max],0.001*rz_max,0,4);
    Wr_z.InputName = 'ref(3)';
    Wr_z.OutputName = 'ref_tilde(3)';

%% U filters

% Thrust input
T_max = 2^16-1; %[N]
f_T = 50; %[Hz]
Wu_T = makeweight(T_max,[f_T*2*pi,db2mag(-3)*T_max],0.001*T_max);
    Wu_T.OutputName = 'T';
    Wu_T.InputName = 'T_tilde';

M1_max = (2^16-1)/2; %[N]
f_M1 = 50; %[Hz]
Wu_M1 = makeweight(M1_max,[f_M1*2*pi,db2mag(-3)*M1_max],0.001*M1_max);
    Wu_M1.OutputName = 'M1';
    Wu_M1.InputName = 'M1_tilde';
% Assuming that M2 has same limitations as M1
Wu_M2 = makeweight(M1_max,[f_M1*2*pi,db2mag(-3)*M1_max],0.001*M1_max);
    Wu_M2.OutputName = 'M2';
    Wu_M2.InputName = 'M2_tilde';

M3_max = (2^16-1)/2; %[N]
f_M3 = 50; %[Hz]
Wu_M3 = makeweight(M3_max,[f_M3*2*pi,db2mag(-3)*M3_max],0.001*M3_max);
    Wu_M3.OutputName = 'M3';
    Wu_M3.InputName = 'M3_tilde';

% Disturbance
W_du1 = makeweight(db2mag(-70),[0.05*2*pi,db2mag(-80)],db2mag(-90));W_du1.OutputName = 'd(1)';W_du1.InputName = 'd_tilde(1)';
W_du2 = makeweight(db2mag(-70),[0.05*2*pi,db2mag(-80)],db2mag(-90));W_du2.OutputName = 'd(2)';W_du2.InputName = 'd_tilde(2)';
W_du3 = makeweight(db2mag(-70),[0.05*2*pi,db2mag(-80)],db2mag(-90));W_du3.OutputName = 'd(3)';W_du3.InputName = 'd_tilde(3)';
W_du4 = makeweight(db2mag(-70),[0.05*2*pi,db2mag(-80)],db2mag(-90));W_du4.OutputName = 'd(4)';W_du4.InputName = 'd_tilde(4)';

Wd = blkdiag(W_du1,W_du2,W_du3,W_du4);

%% Combined input filters

Wu_cont = ss(0,zeros(1,4),zeros(4,1),eye(4));
Wu_tot = blkdiag(Wr_x,Wr_y,Wr_z,Wd,Wu_cont);


%% Output filters

%% error filters
ex_max = 10; %[dB]
% f_ex = 100; %[Hz]
f_ex = 10; %[Hz]
We_x = 1/makeweight(0.00001,[f_ex*2*pi,db2mag(-3+ex_max)],db2mag(ex_max));
    We_x.InputName = 'e(1)';
    We_x.OutputName = 'e_tilde(1)';

We_y = We_x;
    We_y.InputName = 'e(2)';
    We_y.OutputName = 'e_tilde(2)';

ez_max = 10; %[N]
f_ez = 5; %[Hz]
We_z = 1/makeweight(0.00001,[f_ez*2*pi,db2mag(-3+ez_max)],db2mag(ez_max));
    We_z.InputName = 'e(3)';
    We_z.OutputName = 'e_tilde(3)';

%% Complementary sensitivity
% Translations
CS_x_max = 10; %[]
f_CSx = 10; %[Hz]
Wy_x = 1/makeweight(db2mag(CS_x_max),[f_CSx*2*pi,db2mag(-3+CS_x_max)],0.001);
    Wy_x.InputName = 'x';
    Wy_x.OutputName = 'x_tilde';

Wy_y = Wy_x;
    Wy_y.InputName = 'y';
    Wy_y.OutputName = 'y_tilde';

CS_z_max = 10; %[]
f_CSz = 10; %[Hz]
Wy_z = 1/makeweight(db2mag(CS_z_max),[f_CSx*2*pi,db2mag(-3+CS_z_max)],0.001);
    Wy_z.InputName = 'z';
    Wy_z.OutputName = 'z_tilde';

% Rotations
CS_phi_max = 11; %[]
f_CSphi = 10; %[Hz]
Wy_phi = 1/makeweight(db2mag(CS_phi_max),[f_CSphi*2*pi,db2mag(-3+CS_phi_max)],0.001);
    Wy_phi.InputName = 'phi';
    Wy_phi.OutputName = 'phi_tilde';

Wy_theta = Wy_phi;
    Wy_theta.InputName = 'theta';
    Wy_theta.OutputName = 'theta_tilde';

CS_psi_max = 11; %[]
f_CSpsi = 10; %[Hz]
Wy_psi = 1/makeweight(db2mag(CS_psi_max),[f_CSpsi*2*pi,db2mag(-3+CS_psi_max)],0.001);
    Wy_psi.InputName = 'psi';
    Wy_psi.OutputName = 'psi_tilde';

%% U output filters
Wuy_T = 1/Wu_T;    
    Wuy_T.InputName = 'T';
    Wuy_T.OutputName = 'T_tilde';
Wuy_M1 = 1/Wu_M1;    
    Wuy_M1.InputName = 'M1';
    Wuy_M1.OutputName = 'M1_tilde';
Wuy_M2 = 1/Wu_M2;    
    Wuy_M2.InputName = 'M2';
    Wuy_M2.OutputName = 'M2_tilde';
Wuy_M3 = 1/Wu_M3;    
    Wuy_M3.InputName = 'M3';
    Wuy_M3.OutputName = 'M3_tilde';

%% Combined output filters

Wy_cont = ss(0,zeros(1,6),zeros(6,1),eye(6));
Wy_tot = blkdiag(We_x,We_y,We_z,Wy_x,Wy_y,Wy_z,Wy_phi,Wy_theta,Wy_psi,Wuy_T,Wuy_M1,Wuy_M2,Wuy_M3,Wy_cont);

save('./_weights/Wfilters1.mat','Wy_tot','Wu_tot');
end

for i=1 % Weigthing filters for uncertain plant 1
%% Input filters
%% Ref filters

% Reference input
rx_max = 1; %[m]
f_rx = 0.145; %[Hz]
Wr_x = makeweight(rx_max,[f_rx*2*pi,db2mag(-3)*rx_max],0.001*rx_max,0,4);
    Wr_x.InputName = 'ref(1)';
    Wr_x.OutputName = 'ref_tilde(1)';

% Assuming that y has same limitations as x
% Wr_y = Wr_x;
%     Wr_y.InputName = 'ref(2)';
%     Wr_y.OutputName = 'ref_tilde(2)';
ry_max = 0.5; %[m]
f_ry = 0.145; %[Hz]
Wr_y = makeweight(ry_max,[f_ry*2*pi,db2mag(-3)*ry_max],0.001*ry_max,0,4);
    Wr_y.InputName = 'ref(2)';
    Wr_y.OutputName = 'ref_tilde(2)';

rz_max = 1.5; %[m]
f_rz = 0.145; %[Hz]
Wr_z = makeweight(rz_max,[f_rz*2*pi,db2mag(-3)*rz_max],0.001*rz_max,0,4);
    Wr_z.InputName = 'ref(3)';
    Wr_z.OutputName = 'ref_tilde(3)';

%% U filters

% Thrust input
T_max = 2^16-1; %[N]
f_T = 50; %[Hz]
Wu_T = makeweight(T_max,[f_T*2*pi,db2mag(-3)*T_max],0.001*T_max);
    Wu_T.OutputName = 'T';
    Wu_T.InputName = 'T_tilde';

M1_max = (2^16-1)/2; %[N]
f_M1 = 50; %[Hz]
Wu_M1 = makeweight(M1_max,[f_M1*2*pi,db2mag(-3)*M1_max],0.001*M1_max);
    Wu_M1.OutputName = 'M1';
    Wu_M1.InputName = 'M1_tilde';
% Assuming that M2 has same limitations as M1
Wu_M2 = makeweight(M1_max,[f_M1*2*pi,db2mag(-3)*M1_max],0.001*M1_max);
    Wu_M2.OutputName = 'M2';
    Wu_M2.InputName = 'M2_tilde';

M3_max = (2^16-1)/2; %[N]
f_M3 = 50; %[Hz]
Wu_M3 = makeweight(M3_max,[f_M3*2*pi,db2mag(-3)*M3_max],0.001*M3_max);
    Wu_M3.OutputName = 'M3';
    Wu_M3.InputName = 'M3_tilde';

% Disturbance
W_du1 = makeweight(db2mag(-70),[0.05*2*pi,db2mag(-80)],db2mag(-90));W_du1.OutputName = 'd(1)';W_du1.InputName = 'd_tilde(1)';
W_du2 = makeweight(db2mag(-70),[0.05*2*pi,db2mag(-80)],db2mag(-90));W_du2.OutputName = 'd(2)';W_du2.InputName = 'd_tilde(2)';
W_du3 = makeweight(db2mag(-70),[0.05*2*pi,db2mag(-80)],db2mag(-90));W_du3.OutputName = 'd(3)';W_du3.InputName = 'd_tilde(3)';
W_du4 = makeweight(db2mag(-70),[0.05*2*pi,db2mag(-80)],db2mag(-90));W_du4.OutputName = 'd(4)';W_du4.InputName = 'd_tilde(4)';

Wd = blkdiag(W_du1,W_du2,W_du3,W_du4);

%% Combined input filters

Wu_cont = ss(0,zeros(1,4),zeros(4,1),eye(4));
Wu_tot = blkdiag(Wr_x,Wr_y,Wr_z,Wd,Wu_cont);


%% Output filters

%% error filters
ex_max = 10; %[dB]
% f_ex = 100; %[Hz]
f_ex = 10; %[Hz]
We_x = 1/makeweight(0.00001,[f_ex*2*pi,db2mag(-3+ex_max)],db2mag(ex_max));
    We_x.InputName = 'e(1)';
    We_x.OutputName = 'e_tilde(1)';

We_y = We_x;
    We_y.InputName = 'e(2)';
    We_y.OutputName = 'e_tilde(2)';

ez_max = 10; %[N]
f_ez = 5; %[Hz]
We_z = 1/makeweight(0.00001,[f_ez*2*pi,db2mag(-3+ez_max)],db2mag(ez_max));
    We_z.InputName = 'e(3)';
    We_z.OutputName = 'e_tilde(3)';

%% Complementary sensitivity
% Translations
CS_x_max = 10; %[]
f_CSx = 10; %[Hz]
Wy_x = 1/makeweight(db2mag(CS_x_max),[f_CSx*2*pi,db2mag(-3+CS_x_max)],0.001);
    Wy_x.InputName = 'x';
    Wy_x.OutputName = 'x_tilde';

Wy_y = Wy_x;
    Wy_y.InputName = 'y';
    Wy_y.OutputName = 'y_tilde';

CS_z_max = 10; %[]
f_CSz = 10; %[Hz]
Wy_z = 1/makeweight(db2mag(CS_z_max),[f_CSx*2*pi,db2mag(-3+CS_z_max)],0.001);
    Wy_z.InputName = 'z';
    Wy_z.OutputName = 'z_tilde';

% Rotations
CS_phi_max = 20; %[]
f_CSphi = 10; %[Hz]
Wy_phi = 1/makeweight(db2mag(CS_phi_max),[f_CSphi*2*pi,db2mag(-3+CS_phi_max)],0.001);
    Wy_phi.InputName = 'phi';
    Wy_phi.OutputName = 'phi_tilde';

Wy_theta = Wy_phi;
    Wy_theta.InputName = 'theta';
    Wy_theta.OutputName = 'theta_tilde';

CS_psi_max = 20; %[]
f_CSpsi = 10; %[Hz]
Wy_psi = 1/makeweight(db2mag(CS_psi_max),[f_CSpsi*2*pi,db2mag(-3+CS_psi_max)],0.001);
    Wy_psi.InputName = 'psi';
    Wy_psi.OutputName = 'psi_tilde';

%% U output filters
Wuy_T = 1/Wu_T;    
    Wuy_T.InputName = 'T';
    Wuy_T.OutputName = 'T_tilde';
Wuy_M1 = 1/Wu_M1;    
    Wuy_M1.InputName = 'M1';
    Wuy_M1.OutputName = 'M1_tilde';
Wuy_M2 = 1/Wu_M2;    
    Wuy_M2.InputName = 'M2';
    Wuy_M2.OutputName = 'M2_tilde';
Wuy_M3 = 1/Wu_M3;    
    Wuy_M3.InputName = 'M3';
    Wuy_M3.OutputName = 'M3_tilde';

%% Combined output filters

Wy_cont = ss(0,zeros(1,6),zeros(6,1),eye(6));
Wy_tot = blkdiag(We_x,We_y,We_z,Wy_x,Wy_y,Wy_z,Wy_phi,Wy_theta,Wy_psi,Wuy_T,Wuy_M1,Wuy_M2,Wuy_M3,Wy_cont);

save('./_weights/unc_Wfilters1.mat','Wy_tot','Wu_tot');
end




















