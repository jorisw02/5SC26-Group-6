function [OLsys,G,G_all] = makeUncLinModel(m,I)
%% Load variables
g = 9.81;
inv_I = inv(I);
load('sysParam.mat')

%%
syms x y z
syms x_dot y_dot z_dot
syms phi theta varPsi
syms p q r

states = ([x;...
          y;...
          z;...
          x_dot;...
          y_dot;...
          z_dot;...
          phi;...
          theta;...
          varPsi;...
          p;...
          q;...
          r]);

syms T M1 M2 M3
u = ([T ;...
     M1;...
     M2;...
     M3]);

%% Input change
syms cmd1 cmd2 cmd3 cmd4
syms ctrlT ctrRoll ctrPitch ctrlYaw
u = ([ctrlT ;...
      ctrRoll;...
      ctrPitch;...
      ctrlYaw]);

cmd1 = ctrlT-0.5*ctrRoll+0.5*ctrPitch+ctrlYaw;
cmd2 = ctrlT-0.5*ctrRoll-0.5*ctrPitch-ctrlYaw;
cmd3 = ctrlT+0.5*ctrRoll-0.5*ctrPitch+ctrlYaw;
cmd4 = ctrlT+0.5*ctrRoll+0.5*ctrPitch-ctrlYaw;

Kf2 = 2.130295e-11;
Kf1 = 1.032633e-6;
Kf0 = 5.484560e-4;

Km1 = 0.005964552;
Km0 = 1.563383;

F1 = Kf2*cmd1.^2+Kf1*cmd1+Kf0;
F2 = Kf2*cmd2.^2+Kf1*cmd2+Kf0;
F3 = Kf2*cmd3.^2+Kf1*cmd3+Kf0;
F4 = Kf2*cmd4.^2+Kf1*cmd4+Kf0;
T1 = Km1*F1+Km0;
T2 = Km1*F2+Km0;
T3 = Km1*F3+Km0;
T4 = Km1*F4+Km0;

eqn2_1 = T == sum([F1 F2 F3 F4]);
% eqn2_2 = M1 == sqrt(2)*L*((F1+F2)-(F3+F4));
% eqn2_3 = M2 == sqrt(2)*L*((F2+F3)-(F1+F4));
eqn2_2 = M1 == L*(F2-F4);
eqn2_3 = M2 == L*(F3-F1);
eqn2_4 = M3 == T1-T2+T3-T4;

%% Equilibrium point
states_eq = ([0 ;...
             0 ;...
             0 ;...
             0 ;...
             0 ;...
             0 ;...
             0 ;...
             0 ;...
             0 ;...
             0 ;...
             0 ;...
             0 ]);

% Calculate the equilibrium cmd
cmd_eq = getEqCMD(m,g);
u_eq = ([cmd_eq;...
        0   ;...
        0   ;...
        0   ]);

%% Equations of motion

Q = [1 sin(phi)*tan(theta) cos(phi)*tan(theta);0 cos(phi) -sin(phi);0 sin(phi)/cos(theta) cos(phi)/cos(theta)];
R = rotation_matrix(phi,theta,varPsi);

dx = ([x_dot;... %Begin of translation EoM
      y_dot;...
      z_dot;...
      0;...
      0;...
      -g;... %End of translation EoM
      Q*[p;q;r];... %Begin of rotation EoM
      -inv_I*(cross([p;q;r],I*[p;q;r]))]+... %End of rotation EoM
      [0;...
       0;...
       0;...
       1/m*R*[0;0;T];...
       0;...
       0;...
       0;...
       inv_I*[M1;M2;M3]]);

%% Add the new input to the system
dx = subs(dx,lhs(eqn2_1),rhs(eqn2_1));
dx = subs(dx,lhs(eqn2_2),rhs(eqn2_2));
dx = subs(dx,lhs(eqn2_3),rhs(eqn2_3));
dx = subs(dx,lhs(eqn2_4),rhs(eqn2_4));
% 
% dx = subs(dx,lhs(eqn1_1),rhs(eqn1_1));
% dx = subs(dx,lhs(eqn1_2),rhs(eqn1_2));
% dx = subs(dx,lhs(eqn1_3),rhs(eqn1_3));
% dx = subs(dx,lhs(eqn1_4),rhs(eqn1_4));

%% Outputs of motion

outs = [-x;...
        y;...
        z;...
        180/pi*phi;...
        180/pi*theta;...
        180/pi*varPsi];

outs_allstate = [-x;...
                  y;...
                  z;...
                 -x_dot;...
                  y_dot;...
                  z_dot;...
                  180/pi*phi;...
                  180/pi*theta;...
                  180/pi*varPsi;...
                  180/pi*Q*[p;q;r]];

%% Jacobians
A_deriv = jacobian(dx,states);
B_deriv = jacobian(dx,u);
C_deriv = jacobian(outs,states);
C_allstate_deriv = jacobian(outs_allstate,states);
D_deriv = jacobian(outs,u);
D_allstate_deriv = jacobian(outs_allstate,u);

%% Linear state space matrices
A = double(subs(A_deriv,[states;u],[states_eq;u_eq]));
B = double(subs(B_deriv,[states;u],[states_eq;u_eq]));
C = double(subs(C_deriv,[states;u],[states_eq;u_eq]));
C_allstate = double(subs(C_allstate_deriv,[states;u],[states_eq;u_eq]));
D = double(subs(D_deriv,[states;u],[states_eq;u_eq]));
D_allstate = double(subs(D_allstate_deriv,[states;u],[states_eq;u_eq]));

I_unc1 = ureal('I1',B(10,2),'Percentage',5);
I_unc2 = ureal('I2',B(11,3),'Percentage',5);
I_unc3 = ureal('I3',B(12,4),'Percentage',5);

I_unc12 = ureal('I12',0,'PlusMinus',1e-6);
I_unc13 = ureal('I13',0,'PlusMinus',1e-6);
I_unc23 = ureal('I23',0,'PlusMinus',1e-6);

I_unc = [I_unc1 I_unc12 I_unc13;
         I_unc12 I_unc1 I_unc23;
         I_unc13 I_unc23 I_unc3];

m_unc = ureal('m',B(6,1),'Percentage',5);%B(6,1);%
B(6,1) = m_unc;

B = [zeros(5,4);m_unc zeros(1,3);zeros(3,4);zeros(3,1) I_unc];

G = ss(A,B,C,D);
G_all = ss(A,B,C_allstate,D_allstate);

% G.InputName  = {'T','M1','M2','M3'};
G.InputName  = {'Ug'};
% G.InputName  = {'ThurstCmd','RollCmd','PitchCmd','YawCmd'};
G.outputName = {'x','y','z','phi','theta','psi'};
%% Sytem connection

% plantInputName  = {'ref','d','T','M1','M2','M3'};
plantInputName  = {'ref','d','ThurstCmd','RollCmd','PitchCmd','YawCmd'};
plantOutputName = {'e','x','y','z','phi','theta','psi','ThurstCmd','RollCmd','PitchCmd','YawCmd','e','phi','theta','psi'};

sumRef = 'e = ref - %y';
signames1 = G.outputName([1:3]);
S1 = sumblk(sumRef, signames1);

sumDistrb = 'Ug = d + %yk'; 
% signames2 = {'T','M1','M2','M3'};
signames2 = {'ThurstCmd','RollCmd','PitchCmd','YawCmd'};
S2 = sumblk(sumDistrb, signames2);

OLsys = connect(G,S1,S2,plantInputName,plantOutputName);

end